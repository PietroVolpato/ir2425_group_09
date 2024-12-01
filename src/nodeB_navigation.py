#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class NavigationNode:
    def __init__(self):
        rospy.init_node('nodeB_navigation', anonymous=True)

        # Publisher to send feedback about the navigation
        self.feedback_pub = rospy.Publisher('/navigation_feedback', String, queue_size=10)

        # Subscriber to get the computed goal from nodeB_exploration
        self.goal_sub = rospy.Subscriber('/exploration_goal', PoseStamped, self.goal_callback)

        # Action client to communicate with move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")

        # Internal state to store the current goal
        self.current_goal = None

    def goal_callback(self, goal):
        """Receive and send exploration goals to the Navigation Stack using SimpleActionClient."""
        self.current_goal = goal  # Store the current goal

        # Create a MoveBaseGoal
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header = goal.header
        move_base_goal.target_pose.pose = goal.pose

        # Send the goal to move_base
        rospy.loginfo("Sending goal to move_base")
        self.client.send_goal(move_base_goal, feedback_cb=self.feedback_callback)

        # Optionally, wait for result or handle it asynchronously
        self.client.wait_for_result()
        result = self.client.get_state()

        if result == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached")
            self.feedback_pub.publish("goal reached")
        else:
            rospy.loginfo("Goal not reached")
            self.feedback_pub.publish("goal not reached")

    def feedback_callback(self, feedback):
        """Handle feedback from move_base (optional)."""
        # Process feedback if needed
        pass

if __name__ == '__main__':
    try:
        node = NavigationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
