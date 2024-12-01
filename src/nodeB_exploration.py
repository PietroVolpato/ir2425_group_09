#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import math
import numpy as np

class ExplorationNode:
    def __init__(self):
        rospy.init_node('exploration_node', anonymous=True)

        # Publisher to send goals
        self.goal_pub = rospy.Publisher('/exploration_goal', PoseStamped, queue_size=10)

        # Subscriber to get laser scan data
        rospy.Subscriber('/scan_raw', LaserScan, self.laser_callback)

        # Parameters
        self.safe_distance = 0.5

    def laser_callback(self, msg):
        """Process laser data and publish a goal to explore."""
        # Find indices of scan ranges that are safe
        safe_indices = [i for i, distance in enumerate(msg.ranges)
                        if distance > self.safe_distance and not math.isinf(distance)]

        if not safe_indices:
            rospy.loginfo("No safe points detected.")
            return

        # Select a point to navigate to
        chosen_index = np.random.choice(safe_indices)
        angle = msg.angle_min + chosen_index * msg.angle_increment
        distance = msg.ranges[chosen_index]

        # Compute coordinates
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)

        # Create and publish the goal
        goal = PoseStamped()
        goal.header.frame_id = msg.header.frame_id  # Use the same frame as the laser scan
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0  # Facing forward

        self.goal_pub.publish(goal)
        rospy.loginfo(f"Published goal to x: {x:.2f}, y: {y:.2f}")

if __name__ == '__main__':
    try:
        node = ExplorationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass