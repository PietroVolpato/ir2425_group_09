#!/usr/bin/env python3
"""
NAVIGATION NODE

This node is responsible to properly format the goal received by the exploration node subscribing to the /exploration_goal topic, and send the goal to the
ROS navigation stack using a actionlib.SimpleActionClient('move_base', MoveBaseAction).
Other than the standard feedback provided by actionlib.GoalStatus, it is implemented also an alternative logic to monitor the goal, and eventually consider
it as reached, or abort it because the timout was reached.
The logic is implemented in the function monitor_goal() (see below for details), and it is very important since, considering this task of finding objects in an
environment, significantly improve the capability of the robot to explore it.
The fact that a goal is consider 'terminated' can happen for several reasons:
    1) the goal is reached by Actionlib (actionlib.GoalStatus.SUCCEEDED)
    2) the goal is reached by monitor_goal(), because tiago is very close to it.
    3) the goal is aborted by Actionlib (actionlib.GoalStatus.ABORTED OR actionlib.GoalStatus.REJECTED)
    4) the goal is aborted by monitor_goal(), because time has expired
    5) the goal is aborted because the exploration node turned CONTROL LAW MODE ON
    6) the goal is aborted because leading to a collision with the table (see static table assumption)

Once a goal is terminated, a message with the outcome of the goal is sent to the GOAL MANAGEMENT NODE, in such way that can properly handle the situation
and provide precise feedback to NODE A.
...
STATIC TABLE ASSUMPTION. To handle the presence of a table in the map we assume that the table is static, and the coordinates of the four corner points
of the table must be given as parameters. Such measured points (map frame) are, with a slight exceed of the area, from bottom right corner clockwise:
    A = (6.6, -1.6)
    B = (7.8, -1.6)
    C = (7.8, -3)
    D = (6.6, -3)
We can then compute four coordinates: x1, x2, y1, y2, inferred from the 4 corners plus a safe distance (self.safe_distance_table), since Tiago occupies a non
negligible area in the map. Then a point (x,y) is inside the "table area" iff x1 <= x <= x2 and y1 <= y <= y2.
If tiago is detected to be just entered this area, we cancel the current goal (hence stop), and set as goal a proper "escape point". There are 2 escape points:
one on the region bottom left of the table (escape_point_bottom), the other on the region top left of the table (escape_point_top). Such points are computed as
function of x1,x2,y1,y2 and were determined empirically. The proper escape point is given as goal based both on the direction from which Tiago was approaching the
table, and on the position of Tiago w.r.t. the table. There is a good chance that from one escape point Tiago reaches the other (cross the table without collision), 
resulting in an efficient exploration of the room. This strategy ensure that Tiago can't collide with the table, despite being an "invisible" obstacle for the Lidar sensor.
Despite being a loss of generality, we decided to pass the coordinates of the table as parameters of the node, in such way that is possible to use this strategy for
any rectangular-shape obstacle.
"""

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import tf2_ros
import tf2_geometry_msgs
import time
import math

class NavigationNode:
    def __init__(self):
        rospy.init_node('nodeB_navigation', anonymous=True)
        rospy.loginfo("Navigation Node initialized.")

        # Action Client for move_base

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server.")

        # Publisher for navigation feedback
        self.feedback_pub = rospy.Publisher('/navigation_feedback', String, queue_size=50)

        # Publisher to send any feedback to node_A (e.g. "task completed")
        self.status_pub = rospy.Publisher('/node_b_feedback', String, queue_size=50)

        # Subscriber to get exploration goals
        rospy.Subscriber('/exploration_goal', PoseStamped, self.goal_callback)

        # subscriber to know if the exploration is on control mode or not
        rospy.Subscriber('/control_law_command', String, self.control_law_callback)

        # publisher to send direct velocity commands (needed only to handle the table)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=50)

        # TF listener 
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Get table corner parameters. Static table assumption
        self.corners = {
            'A': tuple(map(float, rospy.get_param("~table_corners/A").split(','))),
            'B': tuple(map(float, rospy.get_param("~table_corners/B").split(','))),
            'C': tuple(map(float, rospy.get_param("~table_corners/C").split(','))),
            'D': tuple(map(float, rospy.get_param("~table_corners/D").split(','))),
        }

        # quantities to define the table area (larger than the actual table area to ensure safe distance)
        self.safe_distance_table = 0.5

        self.x1 = min(elem[0] for elem in self.corners.values()) - self.safe_distance_table
        self.x2 = max(elem[0] for elem in self.corners.values()) + self.safe_distance_table
        self.y1 = min(elem[1] for elem in self.corners.values()) - self.safe_distance_table
        self.y2 = max(elem[1] for elem in self.corners.values()) + self.safe_distance_table
        # a point (x,y) in the map frame is inside the area of the table, with safe distance, if x1 <= x <= x2 and y1 <= y <= y2

        # definition of the two "escape points"
        self.escape_point_distance = 0.5
        self.escape_point_bottom = (self.x2 + self.escape_point_distance, self.y2 - 0.4)
        self.escape_point_top = (self.x2 + self.escape_point_distance, self.y1 + 0.4)

        # flags to handle the problem to avoid table
        self.approaching_direction = None
        self.chosen_escape_point = None

        self.is_escape_point = False # to keep track if the current goal is an escape point (trated slightly different)

        # to handle timout
        self.goal_start_time = None
        self.goal_timeout = 20.0  # Timeout in seconds

        # to handle when a goal is considered reached.
        self.current_position = None  # The robot's current (x, y) position
        self.goal_position = None
        self.xy_tolerance = 0.35  # Tolerance for x, y position to consider a goal reached

        self.control_law_active = False  # flag for control law mode

    def goal_callback(self, goal_msg):
        """
        This callback activates when a message, containing a goal, is received from the exploration node.
        The goal is sent to the move_base client, using a MoveBaseAction to the ROS navigation stack.
        To handle the execution of the action, done_callback checks the status of the goal through actionlib.
        Furthermore, the function monitor_goal() checks if tiago is close (euclidean distance) from the goal, and in this case the
        goal is also marked as reached (see documentation of monitor_goal). The function monitor_goal() also handles the timout
        """

        if self.control_law_active:  # ensure no goal is given when we are in control law mode
            return
        # Store the goal position on the internal state variable
        self.goal_position = (goal_msg.pose.position.x, goal_msg.pose.position.y)

        # Create a MoveBaseGoal from the received PoseStamped
        goal = MoveBaseGoal()
        goal.target_pose = goal_msg

        # Publish feedback
        x = "{:.3f}".format(goal_msg.pose.position.x)
        y = "{:.3f}".format(goal_msg.pose.position.y)

        # Send the goal to move_base
        self.client.send_goal(goal, done_cb=self.done_callback)
        self.status_pub.publish(String(data=f"Sending GOAL: x={x}, y={y}. Tiago is moving"))
        # Update internal state
        self.goal_start_time = time.time()

        # Monitor goal progress
        self.monitor_goal()
    
    def cancel_goal(self):
        """
        Function to cancel the actual goal given to move base, and reset some flags at the same time
        """
        self.goal_position = None
        self.client.cancel_goal()  # Cancel move_base goal
    
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
            # If current position cannot be determined, fall back to (0, 0)
            return 0.0, 0.0
        
    def monitor_goal(self):
        """
        This function implements the navigation logic, build specifically for this task (general case more as possible).
        The functionalities implemented are 3:
        1) RELAXATION OF "GOAL REACHED". As we know, the ROS navigation stack impose that, to consider a goal reached, Tiago needs
            to reach the exact position with the exact orientation. Such strict requirement is not needed for this task, since we just
            need to generate goals that make Tiago explore the environment. To consider a goal reached this function needs only that tiago
            is close enough to the goal (see self.xy_tolerance), and there are no orientation requirements. 
            This is useful to reduce the probability that a goal fails and avoid losing time to reach the exact position/orientation.
        2) TIMOUT HANDLING. Since might happen that Tiago tries to reach a goal, and some obstacles blocks his way to the end of the goal,
            there is a time limit (see self.goal_timout) that if expires, the goal is cancelled and a new goal is requested. This
            functionality is very important to avoid that Tiago gets stuck for a very long time.
        3) STATIC TABLE ASSUMPTION IMPLEMENTATION. See the documentation at the beginning.
         
        """
        rate = rospy.Rate(10)  # 10 Hz

        # we monitor goals only if there is an active goal
        while not rospy.is_shutdown() and not self.goal_position is None:       

            self.current_position = self.get_current_position()
            #print(f"CURRENT: ({self.current_position[0]:.3f} , {self.current_position[1]:.3f})")

            # Calculate the distance to the goal
            distance_to_goal = math.sqrt((self.current_position[0] - self.goal_position[0])**2 + (self.current_position[1] - self.goal_position[1])**2)

            # REACHED CONDITION RELAXATION
            # Check if the robot is within the distance tolerance, if it is, consider it as reached
            if distance_to_goal <= self.xy_tolerance:
                rospy.loginfo(f"Goal reached manually based on distance: {self.goal_position}")
                self.feedback_pub.publish(String(data="Goal Reached"))
                self.cancel_goal()  # Cancel move_base goal
                return

            # TIMOUT HANDLING
            elapsed_time = time.time() - self.goal_start_time
            if elapsed_time > self.goal_timeout:
                rospy.logwarn("Goal timeout exceeded! Cancelling goal.")
                self.feedback_pub.publish(String(data="Time expired"))
                self.cancel_goal()
                return
            
            # Understand from which direction we are approaching the table
            

            # UNDERSTAND THE DIRECTION WE WILL BE APPROACHING TABLE
            if self.current_position[1] > self.y2 - self.safe_distance_table:  # tiago is in the region below the table
                self.approaching_direction = "bottom" # keep track of approaching direction 
                
            elif self.current_position[1] < self.y1 + self.safe_distance_table: # tiago is in the region above the table
                self.approaching_direction = "top" 

            # tiago aside of the table, and has enough room to pass it
            if self.y1 + self.safe_distance_table <= self.current_position[1] <= self.y2 - self.safe_distance_table:  
                # we fix the best escape point in this case
                if self.approaching_direction == "bottom":
                    self.chosen_escape_point = "top"  # tiago is aside the table and was trying to reach upper region
                else:
                    self.chosen_escape_point = "bottom" #  tiago is aside the table and was trying to reach lower region

            # TABLE PROXIMITY HANDLING. see documentation at beginning about static table assumption.
            if (self.x1 <= self.current_position[0] <= self.x2 and self.y1 <= self.current_position[1] <= self.y2):
                # Publish feedback
                self.status_pub.publish(String(
                    data=f"DETECTED TABLE PROXIMITY"
                ))
                print(f"APPROACHING DIRECTION: {self.approaching_direction}")

                 # make a choice of the escape point based on the direction, if wasn't already fixed
                if self.chosen_escape_point == "":
                    if self.approaching_direction == "bottom":
                        self.chosen_escape_point = "bottom"
                    else:
                        self.chosen_escape_point = "top"
                self.cancel_goal()
                x_goal = 0
                y_goal = 0
                pose = PoseStamped()
                #qrot​=(0,0,sin(a/2​),cos(a/2​))
                # set the escape point
                if self.chosen_escape_point == "top":
                    if self.approaching_direction == "top":
                        self.rotation(math.pi*3/4, -0.8) # rotation is needed to avoid tiago rotates while moving, hittihg table 
                    # set top escape point as goal
                    x_goal = self.escape_point_top[0]
                    y_goal = self.escape_point_top[1]    
                else:
                    if self.approaching_direction == "bottom":  
                        self.rotation(math.pi*3/4, 0.8) # rotation is needed to avoid planner rotates while moving, hittihg table 
                    # set bottom escape point as goal
                    x_goal = self.escape_point_bottom[0]
                    y_goal = self.escape_point_bottom[1]
                self.status_pub.publish(String(
                    data=f"Sending 'TABLE ESCAPE GOAL ({self.chosen_escape_point})': x={x_goal:.3f}, y={y_goal:.3f}."
                ))
                theta = 0
                # set an helpful orientation based from which side we were approaching the table
                if self.approaching_direction == "top": # want tiago to look below
                    if self.chosen_escape_point == "bottom": # passed the table
                        theta = math.pi * 7/18  # 70 deg
                    else:
                        theta = math.pi * 1/2  # 90 deg
                        
                else:
                    if self.chosen_escape_point == "bottom":
                        theta = -math.pi * 1/2 # -90 drg
                    else:
                        theta = -math.pi * 7/18 # -70 deg
                
                # compute quaternion
                pose.pose.orientation.z = math.sin(theta/2)
                pose.pose.orientation.w = math.cos(theta/2)

                # NOTE: when we send an "escape goal" it is not monitored. We need to be exact on reaching the goal, and no need to handle timout
                pose.header.frame_id = "map"
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = x_goal
                pose.pose.position.y = y_goal

                # Neutral orientation (no specific heading)
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                

                goal = MoveBaseGoal()
                goal.target_pose = pose

                
                # Send the goal to move_base
                self.is_escape_point = True # mark the flag escape point
                self.client.send_goal(goal, done_cb=self.done_callback)
                return

            rate.sleep()

    def done_callback(self, status, result):
        """
        Callback for when the move_base action completes.
        A proper message is sent to the goal management node.
        """     
        self.goal_position= None # Clear current goal
        if status == actionlib.GoalStatus.SUCCEEDED:
            if self.is_escape_point:  # communicate that the reached goal is an escape point (particular case)
                self.feedback_pub.publish(String(data="escape point"))
            else:
                self.feedback_pub.publish(String(data="Goal Reached"))
        elif status in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED]:
            self.feedback_pub.publish(String(data="Goal Failed"))
        else:
            self.feedback_pub.publish(String(data="Goal Unknown Status"))
        self.is_escape_point = False # unmark the flag escape point
    
    def control_law_callback(self, msg):
        """
        This function ensure that when the control law is active, there are no active goals sent to the navigation stack,
        resulting in a possible conflict
        """
        command = str(msg.data).lower()
        if command == "control law on":
            self.cancel_goal()
            self.status_pub.publish(String(data=f"NARROW CORRIDOR: CONTROL LAW ON. Canceled current goal."))
            self.control_law_active = True
        if command == 'control law off':
            self.feedback_pub.publish(String(data="control law off"))
            self.control_law_active = False
    
    def rotation(self, angle, theta):
        """
        This function make tiago rotate on himself.
        Angle: total angle of desired rotation
        Theta: angular velocity in rad/s. Positive for counterclockwise, negative for clockwise
        """

        # Define the Twist message for rotation
        rotate_cmd = Twist()
        rotate_cmd.angular.z = theta  # Angular velocity in radians/second (counterclockwise)

        # Calculate rotation duration 
        rotation_duration = angle / abs(rotate_cmd.angular.z) 

        rate = rospy.Rate(10)  
        start_time = rospy.Time.now()
        
        while (rospy.Time.now() - start_time).to_sec() < rotation_duration:
            self.cmd_vel_pub.publish(rotate_cmd)
            rate.sleep()
            
        self.cmd_vel_pub.publish(Twist())  # Publish zero velocity

if __name__ == '__main__':
    try:
        node = NavigationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass