#!/usr/bin/env python3
"""
Navigation Node

This node is responsible for:
    - Receiving exploration goals
    - Sending goals to the move_base navigation stack
    - Responding to "Continue" or "Stop" commands for exploration
    - Publishing feedback on navigation status
"""

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class nodeB_exploration:
    def __init__(self):
        rospy.init_node('nodeB_exploration', anonymous=True)
        rospy.loginfo("Navigation Node initialized.")

        # Subscribers
        rospy.Subscriber('/exploration_command', String, self.command_callback)

        # Publishers
        self.feedback_pub = rospy.Publisher('/navigation_feedback', String, queue_size=10)

        # Action Client for move_base
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server.")

        # State variables
        self.is_active = False
        

    def command_callback(self, msg):
        """
        Handles 'Continue' and 'Stop' commands.
        """
        command = msg.data.lower()
        if command == "continue":
            self.is_active = True
            goal = self.define_goal()  # Send a test goal to check movement
            rospy.loginfo("Sending test goal to move_base...")
            self.move_base_client.send_goal(goal, done_cb=self.goal_done_callback)
            self.feedback_pub.publish(String(data="Navigating to test goal."))

        elif command == "stop":
            rospy.loginfo("Received 'Stop' command.")
            self.is_active = False
            self.cancel_goal()
        else:
            rospy.logwarn(f"Unknown command received: {command}")

    def define_goal(self):
        """
        Sends a simple test goal to the move_base navigation stack.
        """
        if not self.is_active:
            rospy.logwarn("Navigation is inactive. Cannot send goals.")
            return

        # Define a simple goal (e.g., 1 meter forward in the map frame)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 8.0
        goal.target_pose.pose.position.y = 0.0
        #F or a rotation of angle θ around the Z-axis: q=[0,0,sin⁡(θ/2),cos⁡(θ/2)]
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = -0.707
        goal.target_pose.pose.orientation.w = 0.707
        return goal
        
    def cancel_goal(self):
        """
        Cancels the current goal in the move_base navigation stack.
        """
        rospy.loginfo("Cancelling current goal.")
        self.move_base_client.cancel_all_goals()
        self.feedback_pub.publish(String(data="Navigation stopped."))

    def goal_done_callback(self, state, result):
        """
        Callback for when the goal is reached or failed.
        """
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached successfully.")
            self.feedback_pub.publish(String(data="Goal Reached"))
        else:
            rospy.logwarn("Goal failed or cancelled.")
            self.feedback_pub.publish(String(data="Goal Failed"))

        # Reset active state after goal completion
        self.is_active = False

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = nodeB_exploration()
        node.run()
    except rospy.ROSInterruptException:
        pass
