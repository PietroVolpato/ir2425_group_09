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

        # Subscriber to get exploration goals
        rospy.Subscriber('/exploration_goal', PoseStamped, self.goal_callback)

        # Internal state
        self.current_goal = None
        self.goal_history = []

    def goal_callback(self, goal_msg):
        """
        Receives a PoseStamped goal from the exploration node and sends it to move_base.
        """
        # Create a MoveBaseGoal from the received PoseStamped
        goal = MoveBaseGoal()
        goal.target_pose = goal_msg
        # if (len(self.goal_history) < 1):
        #     # Send goal to move_base
        #     rospy.loginfo(f"Sending goal to move_base: x={goal_msg.pose.position.x}, y={goal_msg.pose.position.y}")
        #     self.client.send_goal(goal, done_cb=self.done_callback, feedback_cb=self.feedback_callback)

        #     # Publish feedback
        #     self.feedback_pub.publish(String(data="Goal sent, navigating"))
        #     self.current_goal = goal
        rospy.loginfo(f"Sending goal to move_base: x={goal_msg.pose.position.x}, y={goal_msg.pose.position.y}")
        self.client.send_goal(goal, done_cb=self.done_callback, feedback_cb=self.feedback_callback)
        self.feedback_pub.publish(String(data="Goal sent, navigating"))
        self.current_goal = goal

    def done_callback(self, status, result):
        """
        Callback for when the move_base action completes.
        """
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached successfully.")
            self.feedback_pub.publish(String(data="Goal Reached"))
            self.goal_history.append(self.current_goal)
        elif status in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED]:
            rospy.logwarn("Goal failed!")
            self.feedback_pub.publish(String(data="Goal Failed"))
        else:
            rospy.loginfo("Goal was canceled or completed with an unknown status.")
            self.feedback_pub.publish(String(data="Goal Unknown Status"))

        # Clear current goal
        self.current_goal = None

    def feedback_callback(self, feedback):
        """
        Callback for feedback during goal execution (optional).
        """
        # rospy.loginfo("Navigation in progress...")

    def cancel_goal(self):
        """
        Cancels the current goal in move_base.
        """
        rospy.loginfo("Cancelling current goal.")
        self.client.cancel_goal()
        self.feedback_pub.publish(String(data="Goal Cancelled"))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = NavigationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
