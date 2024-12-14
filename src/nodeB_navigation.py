#!/usr/bin/env python3
"""
Navigation Node with Actionlib

This node uses actionlib to send goals to the move_base action server and monitor their status.
"""

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
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
        self.feedback_pub = rospy.Publisher('/navigation_feedback', String, queue_size=10)

        # Publisher to send any feedback to node_A (e.g. "task completed")
        self.status_pub = rospy.Publisher('/node_b_feedback', String, queue_size=10)

        # Subscriber to get exploration goals
        rospy.Subscriber('/exploration_goal', PoseStamped, self.goal_callback)

        # subscriber to know if the exploration is on control mode or not
        rospy.Subscriber('/control_law_command', String, self.control_law_callback)

        # TF listener 
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # to handle timout
        self.goal_start_time = None
        self.goal_timeout = 25.0  # Timeout in seconds

        # to handle when a goal is considered reached.
        self.current_position = None  # The robot's current (x, y) position
        self.goal_position = None
        self.xy_tolerance = 0.4  # Tolerance for x, y position to consider a goal reached

    def goal_callback(self, goal_msg):
        """
        This callback activates when a message, containing a goal, is received from the exploration node.
        The goal is sent to the move_base client, using a MoveBaseAction to the ROS navigation stack.
        To handle the execution of the action, done_callback checks the status of the goal through actionlib.
        Furthermore, the function monitor_goal() checks if tiago is close (euclidean distance) from the goal, and in this case the
        goal is also marked as reached (see documentation of monitor_goal). The function monitor_goal() also handles the timout
        """

        # Store the goal position on the internal state variable
        self.goal_position = (goal_msg.pose.position.x, goal_msg.pose.position.y)

        # Create a MoveBaseGoal from the received PoseStamped
        goal = MoveBaseGoal()
        goal.target_pose = goal_msg

        # Publish feedback
        x = "{:.3f}".format(goal_msg.pose.position.x)
        y = "{:.3f}".format(goal_msg.pose.position.y)
        self.status_pub.publish(String(data=f"Sending GOAL: x={x}, y={y}. Tiago is moving"))

        # Send the goal to move_base
        self.client.send_goal(goal, done_cb=self.done_callback)
        # Update internal state
        self.goal_start_time = time.time()

        # Monitor goal progress
        self.monitor_goal()

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
        This function  provides a different definition of "goal reached" w.r.t. the ROS navigation stack.
        Provides several benefits:
        1) Tiago don't lose time to try to reach the specified x,y with a precise orientation. (when you send a goal
           through stack, you are forced to specify a precise oreintation)
        2) Sometimes tiago gets very close to a goal, but he is not able to reach it because the ROS stack deny tiago from getting too close to obstacles.
           To solve this issue, we consider a goal reached if tiago is fairly close to it (see self.xy_tolerance)
           The actual goal sent to the ROS navigation stack is hence deleted, and the message "goal reached" is sent to the goal management node.
           In such way we are able to baypass the fact that for the ROS stack a goal needs to be precisely reached, which is absolutely not necessary
           in this specific task.  

        This function also handles the timeout: if a goal take too much time (see self.goal_timeout), then the goal is canceled, and the message
        "time expired" is sent to the goal management node. 
        """

        rate = rospy.Rate(5)  # 10 Hz
        while not rospy.is_shutdown():
            if self.goal_position is None:
                return        
            self.current_position = self.get_current_position()
            print(f"current: {self.current_position}. GOAL: {self.goal_position}")
            # Check if there is a valid goal and position data
            if self.current_position is None or self.goal_position is None:
                rate.sleep()
                continue

            # Calculate the distance to the goal
            distance_to_goal = math.sqrt((self.current_position[0] - self.goal_position[0])**2 + (self.current_position[1] - self.goal_position[1])**2)

            # Check if the robot is within the distance tolerance, if it is, consider it as reached
            if distance_to_goal <= self.xy_tolerance:
                rospy.loginfo(f"Goal reached manually based on distance: {self.goal_position}")
                self.feedback_pub.publish(String(data="Goal Reached"))
                self.client.cancel_goal()  # Cancel move_base goal
                self.goal_position = None
                return

            # Check for goal timeout
            elapsed_time = time.time() - self.goal_start_time
            if elapsed_time > self.goal_timeout:
                rospy.logwarn("Goal timeout exceeded! Cancelling goal.")
                self.feedback_pub.publish(String(data="Time expired"))
                self.client.cancel_goal()
                self.goal_position = None
                return

            rate.sleep()

    def done_callback(self, status, result):
        """
        Callback for when the move_base action completes.
        """     
        self.goal_position= None # Clear current goal
        if status == actionlib.GoalStatus.SUCCEEDED:
            self.feedback_pub.publish(String(data="Goal Reached"))
        elif status in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED]:
            self.feedback_pub.publish(String(data="Goal Failed"))
        else:
            self.feedback_pub.publish(String(data="Goal Unknown Status"))

    
    def control_law_callback(self, msg):
        command = str(msg.data).lower()
        if command == "control law on":
            self.client.cancel_goal()
            self.goal_position = None
            self.status_pub.publish(String(data=f"CANCELLING CURRENT GOAL: control law mode on"))
        if command == 'control law off':
            self.feedback_pub.publish(String(data="control law off"))

if __name__ == '__main__':
    try:
        node = NavigationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
