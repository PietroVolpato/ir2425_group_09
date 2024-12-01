#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray

class NavigationNode:
    def __init__(self):
        rospy.init_node('nodeB_navigation', anonymous=True)

        # Publisher to send to the navigation stack the goal, which is computed by the nodeB_exploration
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # Publisher to send feedbacks about the navigation (e.g. "goal reached")
        self.feedback_pub = rospy.Publisher('/navigation_feedback', String, queue_size=10)

        # Subscriber to get the computed goal to where send Tiago, from nodeB_exploration
        self.goal_sub = rospy.Subscriber('/exploration_goal', PoseStamped, self.goal_callback)

        # subscriber to get the status feedback from the navigation stack
        self.status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)

        # Internal state: contains the current goal and if a goal is reached/aborted will be the new next goal
        self.current_goal = None

    def goal_callback(self, goal):
        """Receive and send exploration goals to the Navigation Stack."""
        self.current_goal = goal  # set internal state to current goal
        self.goal_pub.publish(goal)  # send the goal to the ROS navigation stack
        self.feedback_pub.publish(String(data="Goal sent, navigating"))   # publish the feedback: navigating to goal 
        rospy.loginfo(f"Sent goal to move_base: x={goal.pose.position.x}, y={goal.pose.position.y}")  # log to display where tiago was sent

    def status_callback(self, status_msg):
        """Monitor navigation progress via move_base status."""
        if not self.current_goal:   # there is no goal set yet
            return

        if status_msg.status_list:
            status = status_msg.status_list[-1].status  # Check the last status
            if status == 3:  # Goal reached
                rospy.loginfo("Goal reached!")
                self.feedback_pub.publish(String(data="Goal Reached")) # publish the feedback: goal is reached
                self.current_goal = None  # update internal state, a new goal will be set
            elif status in [4, 5]:  # Goal aborted or failed
                rospy.logwarn("Navigation failed!")  
                self.feedback_pub.publish(String(data="Goal Failed"))  # publish the feedback: failed to reach the current goal
                self.current_goal = None
        

if __name__ == '__main__':
    try:
        node = NavigationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
