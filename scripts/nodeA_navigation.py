#!/usr/bin/env python

import rospy
from tiago_iaslab_simulation.srv import Coeffs
from std_msgs.msg import String
from std_msgs.msg import Int32
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf2_ros 
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped, Twist
from ir2425_group_09.msg import PlacingMessage  # custom message
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

class nodeA_navigation:
    def __init__(self):
        rospy.init_node("nodeA_navigation")

        self.head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10) # to move the camera angle

        # publisher to notify nodeB that we reached a docking point, thus we are ready to make detections
        self.detections_cmd = rospy.Publisher('/detections_command', String, queue_size=10)

        # publish the x,y,z of the placing target point in base link, plus the height of the picked object
        self.placing_routine_pub = rospy.Publisher('/placing_routine', PlacingMessage, queue_size=10)
        
        self.cmd_vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10) # publish velocities
        rospy.Subscriber("/mobile_base_controller/cmd_vel", Twist, self.cmd_vel_callback)
        rospy.Subscriber('/mobile_base_controller/odom', Odometry, self.odom_callback)

        self.picking_feedback_sub = rospy.Subscriber('/picking_routine_feedback', Int32, self.object_picked_callback)
        self.picking_feedback_sub = rospy.Subscriber('/placing_routine_feedback', String, self.object_placed_callback)
        rospy.Subscriber('/skip', String, self.kill_docking_point)

        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.docking_points = {} # will be computed on runtime when tiago can see the tables
        self.placing_table_center = None # will be initialized on runtime when tiago can see the tables
        self.picking_table_center = None # will be initialized on runtime when tiago can see the tables
      
        # when in a docking point is not detected any desired object, such point is dropped by the list
        self.alive_pickup_points = [] # will be initialized on runtime when tiago can see the tables 
        self.alive_placement_points = [] # will be initialized on runtime when tiago can see the tables
        self.current_point = None

        self.object_heights = { 1 : 0.1, 2 : 0.1, 3 : 0.1,
                                4 : 0.05, 5 : 0.05, 6 : 0.05,
                                7 : 0.035, 8 : 0.035, 9 : 0.035}

        self.table_side = 0.9

        # line
        self.m = None
        self.q = None
        self.target_points_map_frame = None

        self.counter_placed_objects = 0 # count the placed object. Task finish with 3 object placed

        # variables to handle rotations
        self.current_yaw = 0.0
        self.yaw_tolerance = 0.01 

        self.scan_data = None
        self.exited_corridor = False
        self.last_command_time = None  # time of last velocity command to recognize if tiago is not moving

        rospy.Subscriber('/scan', LaserScan, self.scan_callback) # get lidar scan

        # Control parameters for motion control law navigation
        self.min_distance_obstacle_threshold = 0.3  # the control recognize as in front of an obscacle if central lidar ranges are below
        self.min_distance_wall_threshold = 0.4  # the control law perform recognize as close a wall if is below this distances.
        self.side_clearance = 0.7  # threshold for activating the control law looking distances on left and right side
        self.front_clearence = 1.5 # # threshold for activating the control law: a narrow passage is detected if tiago has some room in front.
        self.linear_speed = 0.55  # Base forward speed (m/s) for control law. This number will be normalized (always lowered) based on the detected distances
        self.angular_speed = 0.2  # Base turning speed (rad/s) for control law. This number will be normalized based on the detected distances
        self.chosen_side = None  #  variable used to handle the case tiago is in front of an obstacle. Once is fixed, we stick to that side to avoid obstacle.
        self.control_law_active = False # keep track is the control mode is active
        self.c_l_threshold = 0.5 # hysterisis thresholding to exit control law

        self.kill = False  # flag to kill the current docking point
    
    def scan_callback(self, msg):
        self.scan_data = msg
        if not self.exited_corridor:  # still have to exit corridor
            self.control_law() 
    
    def cmd_vel_callback(self, msg):
        # Update the last time a command was sent
        self.last_command_time = rospy.Time.now()

    def kill_docking_point(self, msg):
        """
        This function is called when a docking point is skipped, and the flag 'kill' is set to True.
        The current docking point is removed from the list of alive points, and the next point is reached.
        """
        self.kill = True

    def is_stationary(self):
        # Check if no velocity commands have been sent for longer than the threshold
        time_since_command = (rospy.Time.now() - self.last_command_time).to_sec()
        return time_since_command > 0.3

    def rotate_to_yaw(self, target_yaw):
        """
        This function makes tiago rotate around the z axis, until he reached the specified angle 'target_yaw'.
        The target yaw must be specified according to the static reference frame (e.g. map frame).
        The angular speed is reduced when the current oreintation is close to the target orientation,
        in order to gain precision on the final orientation.
        """
        rate = rospy.Rate(50)  # Frequenza di pubblicazione (10 Hz)
        angle = target_yaw - self.current_yaw
        norm_angle = self.normalize_angle(angle)
        sign = math.copysign(1, norm_angle)

        while not rospy.is_shutdown():

            yaw_error = self.normalize_angle(target_yaw - self.current_yaw)
            if abs(yaw_error) < self.yaw_tolerance:  # reached target yaw
                self.cmd_vel_pub.publish(Twist()) 
                break 

            angular_speed = 1 if abs(yaw_error) > math.pi / 5 else 0.3 # lower speed if close to target yaw

            twist_msg = Twist()
            twist_msg.angular.z = angular_speed * sign
            self.cmd_vel_pub.publish(twist_msg)
            rate.sleep()

    @staticmethod
    def normalize_angle(angle):
        """
        Normalize the given angle into an equivalent angle in the interval [-PI, PI]
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def move_straight(self, distance, speed=0.5):
        """
        This function makes tiago move straight by the speicified distance.
        The direction of the linear movement is the current planar orientation of Tiago.
        """
        # Definisci il messaggio Twist
        velocity_msg = Twist()
        velocity_msg.linear.x = speed  # Velocità lineare
        velocity_msg.angular.z = 0.0  # Nessuna rotazione

        # Calcola il tempo necessario per percorrere la distanza
        start_time = rospy.Time.now().to_sec()
        current_distance = 0.0

        rate = rospy.Rate(10)  # Frequenza di pubblicazione (10 Hz)

        while current_distance < distance:
            # Pubblica il messaggio di velocità
            self.cmd_vel_pub.publish(velocity_msg)

            # Calcola la distanza percorsa
            current_time = rospy.Time.now().to_sec()
            current_distance = speed * (current_time - start_time)
            rate.sleep()

        velocity_msg.linear.x = 0.0
        self.cmd_vel_pub.publish(velocity_msg)

    def odom_callback(self, msg):
        """
        Receive from odometry the current yaw of tiago. Needed as control feedback in the function rotate_to_yaw()
        """
        q = msg.pose.pose.orientation
        orientation = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation)

        self.current_yaw = yaw

    def move_to(self, target):
        """
        This function is used to move Tiago between docking points that are reachable by doing a linear movement parallel to an axis (either x or y).
        Before starting the linear movement it is performed a rotation to point Tiago towards the correct direction.
        After that, tiago moves straight by the absolute value of the difference between the current and target point coordinate w.r.t. the considered axis.
        In the case the target point is a docking point, it is also specified a precise orientation to achieve after the linear movement.
        """
        
        target = target.lower().strip()
        if target not in self.docking_points.keys():
            rospy.logerr(f"Unkown target given, goal not defined: {target}")
            return
        
        first_rotation = None
        move_along_axis = "y"  # defines if the linear movement if performed parallel to y or x axis. 

        # defined the desired orientation for docking points
        rotation_map = {
            "picking table front" : math.pi,
            "picking table side" : math.pi/2,
            "picking table behind" : 0,  #math.radians(-5),
            "placing table front" : math.pi,
            "placing table behind" : 0
        }

        # set the first rotation, performed before start moving
        if self.current_point in ["corridor exit", "placing table front"]:
            first_rotation = -math.pi/2
        
        elif self.current_point == "picking table front":
            if target == "placing table front":
                first_rotation = math.pi/2
            elif target == "picking table vert1":
                first_rotation = -math.pi/2
        
        elif self.current_point == "picking table vert1":
            if target in ["picking table front", "placing table front"]:
                first_rotation = math.pi/2
          
            elif target in ["picking table side", "picking table vert2"]:
                first_rotation = math.pi
                move_along_axis = "x"
        
        elif self.current_point == "picking table side":
            move_along_axis = "x"
            if target == "picking table vert1":
                first_rotation = 0
            elif target == "picking table vert2":
                first_rotation = math.pi
        
        elif self.current_point == "picking table vert2":
            if target in ["picking table behind", "placing table behind"]:
                first_rotation = math.pi/2
        
            elif target in ["picking table side", "picking table vert1"]:
                first_rotation = 0            
                move_along_axis = "x"

        elif self.current_point == "picking table behind":
            if target == "picking table vert2":
                first_rotation = -math.pi/2
            elif target == "placing table behind":
                first_rotation = math.pi/2
        
        elif self.current_point == "placing table behind":
            first_rotation = -math.pi/2
        
        if first_rotation is None:
            rospy.logerr(f"The specified target is not reachable from current position with a straight line. {target}")
            return
        
        # s = source t = terminal
        s = self.docking_points[self.current_point] # pick from the dictionary the coordinates of source
        t = self.docking_points[target] # pick from the dictionary the coordinates of target
        
        self.rotate_to_yaw(first_rotation)         # perform the first rotation

        distance = abs(s[1]-t[1]) if move_along_axis == "y" else abs(s[0]-t[0])
        self.move_straight(distance)   # perform the linear movement

        if target in rotation_map.keys():  # specify a precise target orientation only for docking points
            self.rotate_to_yaw(rotation_map[target])
        
        self.current_point = target

        # special case: reached pickup point
        if target in self.alive_pickup_points: 
            rospy.sleep(0.3)  # wait a little bit to stabilize detections
            rospy.loginfo(f"Reached PICKUP POINT {target}")
            self.detections_cmd.publish(String(data="picking"))  # tell nodeB to provide the detections to create collision objects
        
        # special case: reached placement point
        if target in self.alive_placement_points:
            rospy.sleep(0.3)  # wait a little bit to stabilize detections
            rospy.loginfo(f"Reached PLACEMENT POINT {target}")
            self.detections_cmd.publish(String(data="placing"))  # tell nodeB to provide the detections to create collision objects

    def find_path_to_point(self, target_point):
        """
        Given a target_point, this function finds a path from the current point to the target point.
        The returned path is a list of point names, such that consecutive points in the path are reachable
        by performing a linear movement along a single axis (either x or y). The last point of the list is always the target.
        """
        s = self.docking_points[self.current_point] # coordinates starting point
        t = self.docking_points[target_point]  # coordinates of terminal point
        path = []  

        # same horizontal line of target: direct path
        if abs(s[0] - t[0]) < 0.2:  
            path.append(target_point)
            return path
        
        # special case: on the side of picking table
        if self.current_point == "picking table side":
            if s[0] < t[0]:  # tiago is on the side an needs to go in frontal region
                path.append("picking table vert1")
            else : # tiago in on the side and needs to go in the back ragion
                path.append("picking table vert2")
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
        This function takes in input a path (ordered list of points name), obtained by calling find_path_to_point.
        The path is executed by calling the function move_to() on every point in the path.
        Note that consecutive points in the path are always reachable by moving along a single axis.
        """
        for p in path:
            self.move_to(p)
    
    def object_picked_callback(self, msg):
        """
        This function is executed when nodeC_picking_routine has finished, and sent the message with the id of the picked object.
        IF id = -1. This means that there were no target objects in the detections. The current docking point is then remove from the
                    list 'alive_pickup_points', and we proceed to the next pickup point.
        
        ELSE.   The selected placement docking point is the one that, among the alive placement points, is the more convenient.
                It is then computed and executed the path to the placement point using find_path_to_point() and execute_path().
                We then pick the coordinates of the target point on the line equation closer to the docking point, and send them
                to nodeA_placing_routine using a custom message.
        .... TODO ....
        """
        picked_object = msg.data
        if picked_object == -1:
            self.alive_pickup_points.pop(0)  # drop the current docking point
            rospy.loginfo("No desired object detected, moving to next pickup point")
            self.move_to_next_pickup_point()
            return
        
        if self.kill:
            self.alive_pickup_points.pop(0)
            self.kill = False
        
        if len(self.alive_placement_points) == 1:                # chose the only option in this case
            placing_point = self.alive_placement_points[0]
        else:                                                   # 2 options available, chose the more convenient
            if self.current_point == "picking table front":
                placing_point = self.alive_placement_points[0]  # in this case the closest placing point is placing table front
            else:
                placing_point = self.alive_placement_points[1]  # in this case it is more convenient placing table behind

        path = self.find_path_to_point(placing_point)  # find a path to reach the placing table
        self.execute_path(path)  # execute the path

        if self.counter_placed_objects == 0:
            self.target_points_map_frame = self.compute_target_points(self.m, self.q)  # compute target points on the line

        # choose the target point closest to the selected docking placement point
        selected_point = self.target_points_map_frame.pop(0) if placing_point == "placing table front" else self.target_points_map_frame.pop()

        self.check_target_points_feasibility() # preserve feasibility of placement points in self.alive_placement_points

        x,y,z = self.transform_point_to_frame(selected_point, "map", "base_link")

        placing_msg = PlacingMessage()
        # x,y,z of the placing point in base link
        placing_msg.x = x
        placing_msg.y = y
        placing_msg.z = z
        placing_msg.object_height = self.object_heights[picked_object] # height of the picked object

        self.placing_routine_pub.publish(placing_msg) # start the placing routine
    
    def object_placed_callback(self, msg):
        """
        This callbacks plays when the placing routine of an object is completed.
        The counter of placed object is increased by one, and if it is 3 the task is over.
        """
        self.counter_placed_objects += 1 
        if self.counter_placed_objects == 3:
            rospy.loginfo("ALL 3 OBJECTS PLACED, TASK COMPLETED")
        else:
            self.move_to_next_pickup_point()
            rospy.loginfo("Moving to next pickup point")

    def move_to_next_pickup_point(self):
        """
        this function move tiago to the first pickup point in the list 'alive_pickup_points'.
        The path from current position to such point is computed using find_path_to_point(), and executed using execute_path()
        """
        if self.alive_pickup_points == []:
            rospy.loginfo("ALL DOCKING POINTS VISITED, all detected targets were placed.")
            return
        path = self.find_path_to_point(self.alive_pickup_points[0])  # move to the first 'alive' docking point (may contain targets)
        self.execute_path(path)

    def transform_point_to_frame(self, p, old_frame, new_frame):
        """
        This function takes in input:
        p: tuple (x,y,z) representing coordinates of a point
        old_frame: frame in which the coordinates of p are specified
        new_frame: frame in which we want to express the coordinates of p

        return: p' = (x',y',z') coordinates of point p w.r.t. 'new_frame'
        """
        try:
    
            transform = self.tf_buffer.lookup_transform(new_frame, old_frame, rospy.Time(0))
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = old_frame
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
            return x,y,z  # return the coordinates of the point in base link frame

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Transform error: {e}")
     
    
    def get_coefficients(self):
        """
        Get m,q of the line from /straight_line_srv service
        """
        # Wait for the line service to become available
        rospy.loginfo("Waiting for /straight_line_srv service...")
        rospy.wait_for_service("/straight_line_srv")

        # Create a service proxy
        straight_line_srv = rospy.ServiceProxy("/straight_line_srv", Coeffs)
        try: 
            response = straight_line_srv(ready=True)
            m, q = response.coeffs
            return (m, q)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call service /straight_line_srv: %s", str(e))
            rospy.signal_shutdown("Service call failed")
        
    def compute_target_points(self, m, q):
        """
        given the line's m and q (slope and intercept), computes points on the line equation and on the table at a proper distance between each other.
        Use polar coordinates to easly compute points with a specified distance from the line origin.
        Points are specified in map frame, since it is static, and a point will be transformed in base_link when is chosen to place an object.
        This function has to be called when the 'tag_10' transform is available, in order to transform the coordinates of the point in the
        map frame using transform_point_to_frame()
        """
        table_center_map = (self.placing_table_center[0], self.placing_table_center[1], 0)
        x_c, y_c, z_c = self.transform_point_to_frame(table_center_map, "map", "tag_10")  # get table center line frame

        x1 = x_c - self.table_side/2 
        y2 = y_c + self.table_side/2 - 0.08

        distances = np.arange(0, 2, 0.14)  # to modify
        points = []
        a = math.atan(m)
        for r in distances:
            x = r * math.cos(a)
            y = r * math.sin(a) + q
            if x >= x1 and y <= y2:  # point inside the table surface
                x_p, y_p, z_p = self.transform_point_to_frame((x,y,0), "tag_10", "map")  # transform point to map frame, which is always available
                points.append((x_p, y_p, z_p))  # z is 0 in the line reference frame
            else:
                break  # reached table boundary
        rospy.loginfo(f"Computed {len(points)} points on placement table")
        return points

    def check_target_points_feasibility(self):
        """
        This function check the list of available target placing points on the line.
        If the distance between the docking point 'placing table behind' and its closest target point (always the last) is too large,
        then it is infeasible to place an object from behind the table, and 'placing table behind' is dropped from the list 'alive_placement_points'.
        The meaning of this function is to preserve placing feasibility of the points in 'alive_placement_points'.
        """
        if "placing table behind" not in self.alive_placement_points:  # placement point already dropped
            return
        
        feasibility_distance = 0.65

        x_last_point = self.target_points_map_frame[-1][0]
        y_last_point = self.target_points_map_frame[-1][1]
        docking_back = self.docking_points["placing table behind"]

        dist = math.sqrt((x_last_point-docking_back[0])**2 + (y_last_point-docking_back[1])**2)
        if  dist > feasibility_distance:
            self.alive_placement_points.pop(1)  # drop the unfeasible docking point
            rospy.loginfo("REMOVED placement docking point on the back of the table (unfeasible to reach a point on the line)")
        return
    
    def tilt_camera(self, tilt_angle = -0.75):
        """
        Tilts the camera downward by adjusting the head_2_joint.
        param tilt_angle: Angle to tilt the camera (in radians, negative for downward tilt).
        """
        head_cmd = JointTrajectory()
        head_cmd.joint_names = ['head_1_joint', 'head_2_joint']

        # Create a JointTrajectoryPoint for the desired position
        point = JointTrajectoryPoint()
        point.positions = [0.0, tilt_angle]  # Keep head_1_joint neutral, tilt head_2_joint
        point.time_from_start = rospy.Duration(1.0)  # Move in 1 second

        head_cmd.points.append(point)
        self.head_pub.publish(head_cmd)

    def control_law(self):
        """
        This is the control law logic, which is applied if a narrow corridor is detected.
        From the array of Lidar measures, we extract right and left samples, which are all the measurements starting from the two
        extrema of the Lidar covering angle_sides radians. We also extract the front samples, which are the central measurements covering 
        angle_front radians.
        Then if there is few distance on the sides (see side_clearance) and enough room in front (see front_clearence), the control law becomes active.
        There are three modes, listed in order of priority (e.g. mode 3 can't be active if mode 1 or mode 2 conditions are true):
        1) OSTACLE MODE. Active if there is few room in front. To avoid collision linear speed is set to 0 and a proper rotation is performed.
        2) WALL MODE: the distance from the side is low. To avoid collision linear speed is reduced, and a proper rotation is applied to get
            further from the wall
        3) CLEAR MODE: there is enough room both in front and on the sides. The linear speed is kept relatively high, and a small rotation is applied in
            order to try to keep the centre of the corridor.
        """
        if self.scan_data is None:  # not ready to start exploration, or task finished, or problem with lidar data
            return
        # Extract distances to obstacles
        ranges = np.array(self.scan_data.ranges)
        n = len(ranges)
        
        # Filter out invalid or extreme range values
        ranges = np.clip(ranges, 0, 10)  # Assume max range of 10 meters

        angle_sides = math.pi/6  # 30 degrees
        side_samples =  int(angle_sides/self.scan_data.angle_increment)
        angle_front = math.pi/9  # 20 degrees
        front_samples =  int(angle_front/self.scan_data.angle_increment)

        # Get distances to the left, front, and right of the robot
        left_indices = np.arange(n - side_samples, n) % n
        front_indices = np.arange(int(n/2 - front_samples/2), int(n/2 + front_samples/2)) % n
        right_indices = np.arange(0, side_samples) % n

        left_distance = np.mean(ranges[left_indices])
        front_distance = np.mean(ranges[front_indices])
        right_distance = np.mean(ranges[right_indices])

        twist = Twist()

        if self.control_law_active or (left_distance < self.side_clearance and right_distance < self.side_clearance and front_distance > self.front_clearence):
            
            if not self.control_law_active: # give information only when starting the control law
                rospy.loginfo("Control law ON. Traversing corridor")
                self.control_law_active = True

            sign = -1 if right_distance > left_distance else 1  # define clockwise or counterclockwise rotation

            # OBSTACLE MODE. too close to an obstacle in front
            if front_distance < self.min_distance_obstacle_threshold:
                if not self.chosen_side:  # chose one side to take to avoid the obstacle
                    self.chosen_side = "right" if right_distance > left_distance else "left"
                twist.angular.z = -self.angular_speed if self.chosen_side == "right" else self.angular_speed
                twist.linear.x = 0

            # WALL MODE. got close to a wall on the sides
            elif  left_distance < self.min_distance_wall_threshold or right_distance < self.min_distance_wall_threshold:
                twist.angular.z = sign * self.angular_speed
                twist.linear.x = self.linear_speed * min(left_distance, right_distance) / self.side_clearance
                self.chosen_side = None  # there is no obstacle ahead, reset chosen_side
            # CLEAR MODE
            else:    
                twist.linear.x = self.linear_speed
                twist.angular.z = sign * self.angular_speed / 2
                self.chosen_side = None  # there is no obstacle ahead, reset chosen_side

            self.cmd_vel_pub.publish(twist)    # publish the velocity command
        # hysterisis thresholding to exit the control law
        if self.control_law_active and (left_distance >= (self.side_clearance + self.c_l_threshold) or right_distance >= (self.side_clearance + self.c_l_threshold)):
            self.control_law_active = False
            self.exited_corridor = True # flag exit corridor
            rospy.loginfo("Control law OFF. Exited corridor")

    def detect_table_central_supports(self):
        """
        This function must be called only when the lidar ranges cover the table central supports.
        1) we split the lidar ranges into intervals of continous ranges. A continuous range is defined as a sequence of ranges
            with a small difference between consecutive point in the sequence, and a large difference between the adjacent point outside the sequence.
        2) the length of each continuous range is approximated using the length of the arc of the circular sector defined by the lidar ranges covering
            the region.
        3) if the length of a continuous range is in the interval [UB_table_support_lenght, LB_table_support_lenght] then we say that
            such interval of lidar data represents a table central support.
        4) We then consider the central entries of the two continuos ranges representing the tables supports, and transform
            the coordinates of those two points into the static map reference frame.

        This strategy effectively approximates the coordinates of the table centers, whithin a small error.
        """
        rospy.sleep(0.5) # stabilize
        discontinuity_distance = 0.5
        UB_table_support_lenght = 0.15 # assume table support is long at most 15cm
        LB_table_support_lenght = 0.03 # assume table support is long at least 3cm
        angle_min = self.scan_data.angle_min
        angle_increment = self.scan_data.angle_increment

        lidar = np.array(self.scan_data.ranges)
        discontinuities = [] # contains tuples (start_index, number of entries)
        start_index = None
        # look for discontinuities in the lidar tanges
        for i in range(1, len(lidar)):
            diff = abs(lidar[i-1]-lidar[i])
            if diff > discontinuity_distance: # start of a discontinuity
                if start_index is not None:
                    discontinuities.append((start_index,i-start_index))  # register the discontinuity
                start_index = i

        table_center_coordinates = [] # will contain first picking table second placing table
        # filter the discontinuities: length of the arc of the circular sector must be small
        for start, count in discontinuities:
            mid_index = start + int(count/2)
            r_mid = lidar[mid_index]  # r of middle point of the discontinuity
            theta = angle_increment * count # angle of the circular sector of the discontinuity
            # look for short discontinuities (between LB and UB)
            if LB_table_support_lenght <= r_mid * theta <= UB_table_support_lenght: 
                angle = angle_min + mid_index * angle_increment
                # compute x and y of the table leg mid point
                x = r_mid * np.cos(angle)
                y = r_mid * np.sin(angle)
                p = (x,y,0)
                x_map, y_map, z_map = self.transform_point_to_frame(p, self.scan_data.header.frame_id, "map")
                x_map = round(x_map, 3)
                y_map = round(y_map, 3)
                table_center_coordinates.append((x_map, y_map))
        
        self.picking_table_center = table_center_coordinates[0] # (7.953, -2.952)
        self.placing_table_center = table_center_coordinates[1] # (7.928, -1.861)

    def compute_docking_points(self):
        """
        From the (approximated) coordinates of the table centers, we can easily compute the coordinates of the docking points
        and also some transition points. In particular:
        - The X coordinate of the points in the front region of the tables is constant.
        - The X coordinate of the points in the back ragion of the tables is constant.
        - the Y coordinate of the points on the left side region of the pickup table is contant

        Those 3 roperties are crucial and allow us to move between adjacent points by a linear movement along a single axis.

        Furthermore, some points are shifted from the table centers on purpose. For example the placement docking points are
        shifted towards left to be closer to the pickup points and to the line equation.
        """
        if self.picking_table_center is None or self.placing_table_center is None:
            rospy.logerr("Trying to compute docking points, but table centers are not detected yet.")
            return
        x1 = self.picking_table_center[0]  # x of picking table
        y1 = self.picking_table_center[1]  # y of picking table
        x2 = self.placing_table_center[0]  # x of placing table
        y2 = self.placing_table_center[1]  # y of placing table

        # define some offsets to tune the docking points
        d1 = 0.113                      # small offset to define points slightly shifted from table center
        d2 = self.table_side/2 + 0.39   # horizontal (x) offset of front region points 
        d3 = self.table_side/2 + 0.26   # vertical (y) offset of the side points (vertices and picking table side)
        d4 = self.table_side/2 + 0.29   # horizontal (x) offset back region points

        x_avg = (x1+x2)/2 # x should be equal, in practice they have a small difference, we take the avg.

        # placement docking points
        self.docking_points["placing table front"] = (x_avg + d2, y2 - d1*2)
        self.docking_points["placing table behind"] = (x_avg - d4, y2)

        # pickup docking points
        self.docking_points["picking table front"] = (x_avg + d2, y1 + d1 * 2)
        self.docking_points["picking table side"] = (x_avg + d1, y1 - d3)
        self.docking_points["picking table behind"] = (x_avg - d4, y1 + d1)

        # tansition points
        self.docking_points["picking table vert1"] = (x_avg + d2, y1 - d3)
        self.docking_points["picking table vert2"] = (x_avg - d4, y1 - d3)
        self.docking_points["corridor exit"] = (x_avg + d2, y2 + d3)            # staring point

        # initialize all docking point as alive
        self.alive_pickup_points = ["picking table front", "picking table side", "picking table behind"]
        self.alive_placement_points = ["placing table front", "placing table behind"]
    
    def get_current_position(self):
        """
        Retrieves Tiago's current position from TF.
        """
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0))
            position = transform.transform.translation
            return (position.x, position.y)
        except Exception as e:
            rospy.logerr(f"Failed to get current position: {e}")
        
    def run(self):
        rospy.sleep(1.0)
        self.tilt_camera(-1)
        (m, q) = self.get_coefficients()
        self.m = m
        self.q = q
        self.move_straight(1) # move straight to enter corridor and use control law

        rate = rospy.Rate(10)
        while not self.is_stationary():  # Waiting for Tiago to exit the corridor
            rate.sleep()  # Sleep for the remainder of the loop cycle      

        self.detect_table_central_supports()  # obtain the coordinates of table centers
        self.compute_docking_points()         # compute the docking points by adding/subcracting offsets to table centers.

        rospy.loginfo(f"Detected table central supports")
        rospy.loginfo(f"Placement table: ({self.placing_table_center[0]:.3f},{self.placing_table_center[1]:.3f})")
        rospy.loginfo(f"Pickup table: ({self.picking_table_center[0]:.3f},{self.picking_table_center[1]:.3f})")
        rospy.loginfo("Computed docking points from the table centers:")
        for k,v in self.docking_points.items():
            rospy.loginfo(f"{k} : ({v[0]:.3f},{v[1]:.3f})")

        x_curr, y_curr = self.get_current_position()
        x_exit = self.docking_points["corridor exit"][0]
        y_exit = self.docking_points["corridor exit"][1]

        # do some triangular geometry to reach the starting point from the current point
        l1 = abs(y_curr - y_exit)
        l2 = abs(x_curr - x_exit)
        theta = math.atan(l1/l2)        # direction of the linear movement
        theta = -theta if y_exit < y_curr else theta
        ipo = math.sqrt(l1**2 + l2**2)  # lenght of the linear movement

        rospy.loginfo("Moving to starting point (corridor exit)")
        
        self.rotate_to_yaw(theta)
        rospy.loginfo(f"Received line coefficients: m = {m:.4f}, q = {q:.4f}")
        self.move_straight(ipo)
        self.current_point = "corridor exit"  # reached the starting point

        self.move_to_next_pickup_point()  # start  the pick and place loop
        
        rospy.spin()

if __name__ == "__main__":
    try:
        node = nodeA_navigation()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("NodeA terminated.")
