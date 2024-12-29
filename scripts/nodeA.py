#!/usr/bin/env python

import rospy
from tiago_iaslab_simulation.srv import Coeffs  # Replace 'your_package' with your ROS package name
from std_msgs.msg import String
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ir2425_group_09.msg import Detections  # custom message
from tf.transformations import quaternion_from_euler

class NodeA:
    def __init__(self):
        rospy.init_node("nodeA")

        self.head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10) # to move the camera angle

        # publisher to notify nodeB that we reached a docking point, thus we are ready to make detections
        self.ready_detection_pub = rospy.Publisher('/ready_detection', String, queue_size=10)

        # rospy.Subscriber('/detected_objects', Detections, self.process_detections)   # HERE JUST MOMENTARILY, should be in C

        # Initialize actionlib client
        self.nav_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.nav_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server.")

        # Wait for the service to become available
        rospy.loginfo("Waiting for /straight_line_srv service...")
        rospy.wait_for_service("/straight_line_srv")

        # Create a service proxy
        self.straight_line_srv = rospy.ServiceProxy("/straight_line_srv", Coeffs)
        self.target_points = []  # points where to place the objects (line reference frame)
        self.current_navigation_command = None  # keeps track of current navigation command

        # Define static docking points + transition points
        self.docking_points = {
            "corridor exit": (8.7, 0),              # transition point
            "placing table" : (8.7, -2),  
            "picking table front" : (8.8, -3),      # DOCKING POINT
            "picking table vert1" : (9, -4.1),      # transition point bottom left vertex
            "picking table side" : (8, -4.1),         # DOCKING point
            "picking table vert2" : (6.8, -4.1),    # transition point top left vertex
            "picking table behind" : (6.8, -3)      # DOCKING POINT     
        }

        self.alive_docking_points = ["picking table front", "picking table side", "picking table behind"]
        self.current_point = None

    def get_coefficients(self):
        try:            
            response = self.straight_line_srv(ready=True)

            m, q = response.coeffs
            rospy.loginfo(f"Received coefficients: m = {m:.4f}, q = {q:.4f}")
            return m, q

        except rospy.ServiceException as e:
            rospy.logerr("Failed to call service /straight_line_srv: %s", str(e))
            rospy.signal_shutdown("Service call failed")
    
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
            goal.target_pose.pose.orientation.z = 0.7   # look left
            goal.target_pose.pose.orientation.w = -0.7
        elif target == "picking table front":  
            goal.target_pose.pose.orientation.w = 0.087  
            goal.target_pose.pose.orientation.z = 0.99 
        elif target == "placing table" or target == "picking table vert2":
            goal.target_pose.pose.orientation.w = 0  
            goal.target_pose.pose.orientation.z = 1.0  
        elif target == "picking table side":
            goal.target_pose.pose.orientation.w = 0.707  # look right
            goal.target_pose.pose.orientation.z = 0.7  
        elif target == "picking table behind":
            goal.target_pose.pose.orientation.w = -0.99 
            goal.target_pose.pose.orientation.z = -0.047 
      
        #rospy.loginfo(f"Sending goal: {target}")
        self.nav_client.send_goal(goal)

        self.nav_client.wait_for_result()
        result = self.nav_client.get_state()
        
        if result == actionlib.GoalStatus.SUCCEEDED:   
            if target in ["picking table front", "picking table side", "picking table behind"]:  # print when reaching docking points
                rospy.sleep(0.2)  # wait a little bit to stabilize detections
                rospy.loginfo(f"Reached DOCKING POINT {target}")
                self.ready_detection_pub.publish(String(data="ready"))  # tell nodeB to provide the detections
        else:
            rospy.logerr("Navigation FAILED with status: %s", result)
            #self.send_goal(target)

        self.current_point = target

    def set_target_points(self, m, q):
        """
        given the m and q (slope and intercept), computes points on the line equation at a proper distance between each other.
        Use polar coordinates to easly compute points with a specified distance from the line origin.
        """
        distances = [0.2, 0.4, 0.6]
        a = math.atan(m)
        for r in distances:
            x = r * math.cos(a)
            y = r * math.sin(a) + q
            self.target_points.append((x,y))  # save targets in internal state

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

    # def process_detections(self, msg):  # callback to process detections. TEMPORARILY HERE, for testing. SHOULD BE IN NODE C
    #     poses = msg.poses
    #     ids = msg.ids
    #     target = msg.target
    #     print(f"target is : {target}")
    #     print("received detections:")
    #     for i in range(len(poses)):
    #         id = ids[i]
    #         pose = poses[i]
    #         x = pose.position.x
    #         y = pose.position.y
    #         z = pose.position.z
    #         print(f"id: {id}, x = {x:.3f}, y = {y:.3f}, z = {z:.3f}")

    def find_path_to_point(self, target_point):
        """
        This function return a list of name of points, representing the path we need to take to reach the target_point
        form the current position.

        this function will be called in following cases:
        - an object is placed and we need to reach a docking point
        - a docking point dies (no more target obj) and need to reach another one
        - after pick an object, we need to move to a placing position

        The initial point is the current tiago point (internal state)
        target point must be specified as the name of the point (e.g. placing table front)
        """
        s = self.docking_points[self.current_point] # coordinates starting point
        t = self.docking_points[target_point]  # coordinates of terminal point
        path = []  # oc the path does not include the initial (current) point

        if abs(s[1] - t[1]) < 0.2:  # same horizontal line of target: direct path
            path.append(target_point)
            return path
        
        # number that defines the x coordinate of the horizontal line crossing the middle of the tables
        watershed = (self.docking_points["picking table front"][0]+self.docking_points["picking table behind"][0])/2
        region = "front" if s[0] < watershed else "back"

        # define the transition points, the order dipend on the region tiago started in
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

    def run(self):
        rospy.sleep(1.0)
        self.tilt_camera()
        m,q = node.get_coefficients()
        self.set_target_points(m,q)
        #print(self.target_points)
        self.send_goal("corridor exit")        
        self.send_goal("placing table")
        self.send_goal("picking table front")
        self.send_goal("picking table vert1")
        self.send_goal("picking table side")
        self.send_goal("picking table vert2")
        self.send_goal("picking table behind")
        rospy.spin()

if __name__ == "__main__":
    try:
        node = NodeA()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("NodeA terminated.")