#!/usr/bin/env python

import rospy
from tiago_iaslab_simulation.srv import Coeffs
from std_msgs.msg import String
from std_msgs.msg import Int32
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf2_ros 
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped
from ir2425_group_09.msg import PlacingMessage  # custom message

class nodeA_navigation:
    def __init__(self):
        rospy.init_node("nodeA_navigation")

        self.head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10) # to move the camera angle

        # publisher to notify nodeB that we reached a docking point, thus we are ready to make detections
        self.detections_cmd = rospy.Publisher('/detections_command', String, queue_size=10)

        # publish the x,y,z of the placing target point in base link, plus the height of the picked object
        self.placing_routine_pub = rospy.Publisher('/placing_routine', PlacingMessage, queue_size=10) # to move the camera angle

        self.picking_feedback_sub = rospy.Subscriber('/picking_routine_feedback', Int32, self.object_picked_callback)

        # Initialize actionlib client
        self.nav_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.nav_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server.")

        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Define static docking points + transition points
        self.docking_points = {
            "corridor exit": (8.7, 0),              # transition point
            "placing table front" : (8.6, -2),      # DOCKING point (placement)
            "placing table behind" : (6.8, -2),     # DOCKING point (placement)
            "picking table front" : (8.7, -3),      # DOCKING POINT (pickup)
            "picking table vert1" : (9, -4.1),      # transition point bottom left vertex
            "picking table side" : (8, -4.1),       # DOCKING point (pickup)
            "picking table vert2" : (6.8, -4.1),    # transition point top left vertex
            "picking table behind" : (6.8, -3)      # DOCKING POINT (pickup)  
        }

        # list of docking points that may contain a desired object
        # when in a docking point is not detected a desireb object, such point is dropped by the list
        self.alive_pickup_points = ["picking table front", "picking table side", "picking table behind"]
        self.alive_placement_points = ["placing table front", "placing table behind"]
        self.current_point = None

        self.object_heights = { 1 : 0.1, 2 : 0.1, 3 : 0.1,
                                4 : 0.05, 5 : 0.05, 6 : 0.05,
                                7 : 0.035, 8 : 0.035, 9 : 0.035}

        m,q = self.get_coefficients()
        self.target_points_line_frame = self.compute_target_points(m,q) # points where to place the objects (line reference frame)

    def send_goal(self, target):
        """
        Sends a goal using MoveBaseAction. The target should be a name of one of the predefined points, which are associated to 
        predefined coordinates. The target can be either a transition point or a docking point.
        We wait until actionlib communicates that the goal is over, and if the target was a docking point for the picking table,
        a message is sent to nodeB to notify we are ready to make detections.
        """
        goal = MoveBaseGoal()
        target = target.lower().strip()
        if target not in self.docking_points.keys():
            rospy.logerr(f"Unkown target given, goal not defined: {target}")
            return
        
        p = self.docking_points[target] # pick from the dictionary the point corresponding to target
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # set x,y of target point as goal
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]

        # define a proper orientation looking at the table for every docking point
        if target == "corridor exit" or target == "picking table vert1":
            theta = -math.pi/2
        elif target == "picking table front":  
            theta = 16.8/18*math.pi  # look ahead, slightly rotated right          
        elif target == "picking table vert2" or target == "placing table front":  # look ahead
            theta = math.pi  
        elif target == "picking table side":
            theta = math.pi/2  
        elif target == "picking table behind": # look behind, slightly rotated left
            theta = 1/18*math.pi
        elif target == "placing table behind":
            theta = 0  # look behind

        goal.target_pose.pose.orientation.w = math.cos(theta/2)
        goal.target_pose.pose.orientation.z = math.sin(theta/2)
        #rospy.loginfo(f"Sending goal: {target}")
        self.nav_client.send_goal(goal)

        self.nav_client.wait_for_result()
        result = self.nav_client.get_state()
        
        if result == actionlib.GoalStatus.SUCCEEDED:   

            if target in self.alive_pickup_points:  # reached pickup point
                rospy.sleep(0.2)  # wait a little bit to stabilize detections
                rospy.loginfo(f"Reached PICKUP POINT {target}")
                self.detections_cmd.publish(String(data="picking"))  # tell nodeB to provide the detections to create collision objects
            
            if target in self.alive_placement_points:
                rospy.sleep(0.2)  # wait a little bit to stabilize detections
                rospy.loginfo(f"Reached PLACEMENT POINT {target}")
                self.detections_cmd.publish(String(data="placing"))  # tell nodeB to provide the detections to create collision objects
        else:
            rospy.logerr("Navigation FAILED with status: %s", result)

        self.current_point = target  

    def find_path_to_point(self, target_point):
        """
        This function return a list of name of points, representing the path we need to take to reach the target_point
        form the current position.

        this function will be called in following cases:
        - an object is placed and we need to reach a docking point
        - a docking point dies (no more target obj) and need to reach another one
        - after pick an object, we need to move to a placing position

        The initial point is the current tiago point (internal state)
        target point must be specified as the name of the point (e.g. placing table)
        """
        s = self.docking_points[self.current_point] # coordinates starting point
        t = self.docking_points[target_point]  # coordinates of terminal point
        path = []  # oc the path does not include the initial (current) point

        if abs(s[0] - t[0]) < 0.2:  # same horizontal line of target: direct path
            path.append(target_point)
            return path
        
        # number that defines the x coordinate of the horizontal line crossing the middle of the tables
        watershed = (self.docking_points["picking table front"][0]+self.docking_points["picking table behind"][0])/2
        region = "front" if s[0] > watershed else "back"

        # define the transition points, the order depends on the region tiago started in
        if region == "front":
            sequence = ["picking table vert1", "picking table vert2"]
        else:
            sequence = ["picking table vert2", "picking table vert1"]

        path.append(sequence.pop(0))  # add the first transition point

        if target_point == "picking table side":  # we can reach the side from both vertices
            path.append(target_point)
            return path
        
        path.append(sequence.pop(0)) # add second transition point
        path.append(target_point)  # add the goal, in this case to reach the goal is needed to take a path around the table
        return path

    def execute_path(self, path):
        """
        This function takes in input a path (ordered list of points name), and follows the path sending each point
        to the function send_goal().
        You should use this function after calling find_path_to_point(), and tiago must not have moved meanwhile.
        """
        for p in path:
            self.send_goal(p)
    
    def object_picked_callback(self, msg):
        """
        This function is exectured when nodeC_picking_routine has finished, and sent the message with the id of the picked object.
        It is then computed the more convenient placing point and tiago is sent to it.
        .... TODO ....
        """
        picked_object = msg.data

        if len(self.alive_placement_points) == 1:                # chose the only option in this case
            placing_point = self.alive_placement_points[0]
        else:                                                   # 2 options available, chose the more convenient
            if self.current_point == "picking table front":
                placing_point = self.alive_placement_points[0]  # in this case the closest placing point is placing table front
            else:
                placing_point = self.alive_placement_points[1]  # in this case it is more convenient placing table behind

        path = self.find_path_to_point(placing_point)  # find a path to reach the placing table
        self.execute_path(path)  # execute the path

        self.check_target_points_feasibility() # preserve feasibility of placement points in self.alive_placement_points

        # choose the target point closest to the selected docking placement point
        selected_point = self.target_points_line_frame[0] if placing_point == "placing table front" else self.target_points_line_frame[-1]

        target_point = self.transform_target_point_to_frame(selected_point, "base_link")

        placing_msg = PlacingMessage()
        # x,y,z of the placing point in base link
        placing_msg.x = target_point[0]
        placing_msg.y = target_point[1]
        placing_msg.z = target_point[2]
        placing_msg.object_height = self.object_heights[picked_object] # height of the picked object

        self.placing_routine_pub.publish(placing_msg) # start the placing routine

    def move_to_next_pickup_point(self):
        """
        this function move tiago to the next pickup point, which is defined as the first pickup point in the list
        self.alive_pickup_points. Such point has a possibility to contain a target object.
        The path from current position to such point is computed using find_path_to_point(), and executed using execute_path()
        """
        path = self.find_path_to_point(self.alive_pickup_points[0])  # move to the first 'alive' docking point (may contain targets)
        self.execute_path(path)

    def transform_target_point_to_frame(self, p, frame):
        try:
    
            transform = self.tf_buffer.lookup_transform(frame, "tag_10", rospy.Time(0))
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "tag_10"
            pose_stamped.pose.position.x = p[0]
            pose_stamped.pose.position.y = p[1]
            pose_stamped.pose.position.z = p[2]
            pose_stamped.pose.orientation.w = 1  # does not matter

            # Transform the pose to base_link frame
            transformed_pose = do_transform_pose(pose_stamped, transform)

            # Adjust pose to ensure it aligns with the center of the table

            x = transformed_pose.pose.position.x
            y = transformed_pose.pose.position.y
            z = transformed_pose.pose.position.z
            return (x,y,z)  # return the coordinates of the point in base link frame

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Transform error while updating table: {e}")
        except Exception as e:
            rospy.logerr(f"Failed to update table collision object: {e}")
    
    def get_coefficients(self):
        # Wait for the line service to become available
        rospy.loginfo("Waiting for /straight_line_srv service...")
        rospy.wait_for_service("/straight_line_srv")

        # Create a service proxy
        straight_line_srv = rospy.ServiceProxy("/straight_line_srv", Coeffs)
        try: 
            response = straight_line_srv(ready=True)

            m, q = response.coeffs
            rospy.loginfo(f"Received coefficients: m = {m:.4f}, q = {q:.4f}")
            return m, q

        except rospy.ServiceException as e:
            rospy.logerr("Failed to call service /straight_line_srv: %s", str(e))
            rospy.signal_shutdown("Service call failed")
        
    def compute_target_points(self, m, q):
        """
        given the line's m and q (slope and intercept), computes points on the line equation and on the table at a proper distance between each other.
        Use polar coordinates to easly compute points with a specified distance from the line origin.
        Points are specified in map frame, since it is static, and a point is transformed in base_link when is chosen to place an object
        """
        distances = [0.1, 0.2, 0.3]  # to modify
        points = []
        a = math.atan(m)
        for r in distances:
            x = r * math.cos(a)
            y = r * math.sin(a) + q
            points.append((x,y,0))  # z is 0 in the line reference frame
        return points

    def check_target_points_feasibility(self):
        """
        This function check the list of available target placing points on the line.
        If the distance between the docking point 'placing table behind' and its closest target point (is always the last) is too large,
        then it is infeasible to place an object from behind the table, and placing table behind is dropped from the list self.alive_placement_points.
        The same check is applied to the docking point 'placing table front'.
        The meaning of this function is to preserve placing feasibility of the docking placement points in self.alive_placement_points.
        """
        feasibility_distance = 1

        front_point_map = self.transform_target_point_to_frame(self.target_points_line_frame[0], "map")
        back_point_map = self.transform_target_point_to_frame(self.target_points_line_frame[-1], "map")
        docking_front = self.docking_points["placing table front"]
        docking_back = self.docking_points["placing table behind"]

        # check that planar distance is not too large
        if math.sqrt((front_point_map[0]-docking_front[0])**2 + (front_point_map[1]-docking_front[1])**2) > feasibility_distance:
            self.alive_placement_points.pop(0)  # drop the unfeasible docking point
            rospy.loginfo("REMOVED placement docking point in front of the table (unfeasible to reach a target point from there)")
        
        # check that planar distance is not too large
        if math.sqrt((back_point_map[0]-docking_back[0])**2 + (back_point_map[1]-docking_back[1])**2) > feasibility_distance:
            self.alive_placement_points.pop(1)  # drop the unfeasible docking point
            rospy.loginfo("REMOVED placement docking point on the back of the table (unfeasible to reach a target point from there)")
        return
    
    def tilt_camera(self, tilt_angle = -0.75):
        """
        Tilts the camera downward by adjusting the head_2_joint.
        param tilt_angle: Angle to tilt the camera (in radians, negative for downward tilt).
        """
        # Create a JointTrajectory message
        head_cmd = JointTrajectory()
        head_cmd.joint_names = ['head_1_joint', 'head_2_joint']

        # Create a JointTrajectoryPoint for the desired position
        point = JointTrajectoryPoint()
        point.positions = [0.0, tilt_angle]  # Keep head_1_joint neutral, tilt head_2_joint
        point.time_from_start = rospy.Duration(1.0)  # Move in 1 second

        # Add the point to the trajectory
        head_cmd.points.append(point)

        # Publish the command
        self.head_pub.publish(head_cmd)

    def run(self):
        rospy.sleep(1.0)
        self.tilt_camera(-1)

        self.send_goal("corridor exit")
        self.move_to_next_pickup_point()        
        
        rospy.spin()

if __name__ == "__main__":
    try:
        node = nodeA_navigation()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("NodeA terminated.")