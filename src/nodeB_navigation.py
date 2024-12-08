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
import time

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

        # Internal state
        self.current_goal = None
        self.goal_start_time = None
        self.goal_timeout = 30.0  # Timeout in seconds

    def goal_callback(self, goal_msg):
        """
        Receives a PoseStamped goal from the exploration node and sends it to move_base.
        """
        # Create a MoveBaseGoal from the received PoseStamped
        goal = MoveBaseGoal()
        goal.target_pose = goal_msg

        # Send the goal to move_base
        self.client.send_goal(goal, done_cb=self.done_callback, feedback_cb=self.feedback_callback)

        # Publish feedback
        self.status_pub.publish(String(data=f"Goal sent: x={goal_msg.pose.position.x}, y={goal_msg.pose.position.y}. Tiago is moving"))

        # Update internal state
        self.current_goal = goal
        self.goal_start_time = time.time()

    def done_callback(self, status, result):
        """
        Callback for when the move_base action completes.
        """
        if status == actionlib.GoalStatus.SUCCEEDED:
            self.feedback_pub.publish(String(data="Goal Reached"))
            #self.goal_history.append(self.current_goal)
        elif status in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED]:
            self.feedback_pub.publish(String(data="Goal Failed"))
        else:
            self.feedback_pub.publish(String(data="Goal Unknown Status"))

        # Clear current goal
        self.current_goal = None
    
    def feedback_callback(self, feedback):
        """
        Callback to process feedback from move_base.
        The goal is aborted is the elapsed time reaches the timeout
        """
        if self.current_goal and self.goal_start_time:
            elapsed_time = time.time() - self.goal_start_time
            if elapsed_time > self.goal_timeout:
                rospy.logwarn("Goal timed out.")
                self.client.cancel_goal()
                self.goal_start_time = None  # Reset timer
                self.feedback_pub.publish(String(data="Time expired"))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = NavigationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
