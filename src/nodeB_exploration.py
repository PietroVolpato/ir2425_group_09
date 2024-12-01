#!/usr/bin/env python3

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import tf

class ExplorationNode:
    def __init__(self):
        rospy.init_node('simple_exploration_node')

        # Sottoscrizione al topic del LiDAR
        self.lidar_sub = rospy.Subscriber('/scan_raw', LaserScan, self.lidar_callback)

        # Publisher per i goal
        self.goal_pub = rospy.Publisher('/exploration_goal', PoseStamped, queue_size=10)

        # TF listener per trasformazioni
        self.tf_listener = tf.TransformListener()

        # Variabili per i dati del LiDAR
        self.scan_data = None
        self.gap_threshold = 1.0  # Minima distanza tra ostacoli per considerare un gap

    def lidar_callback(self, msg):
        self.scan_data = msg

    def find_best_goal(self):
        """
        Analizza i dati del LiDAR per individuare il punto più distante e libero.
        """
        if self.scan_data is None:
            return None

        ranges = np.array(self.scan_data.ranges)
        angle_min = self.scan_data.angle_min
        angle_increment = self.scan_data.angle_increment

        # Filtra i dati validi
        valid_indices = np.where(ranges > 0)[0]  # Ignora letture non valide (0 o inf)
        if len(valid_indices) == 0:
            return None

        # Trova il punto con la massima distanza
        max_idx = valid_indices[np.argmax(ranges[valid_indices])]
        distance = ranges[max_idx]
        angle = angle_min + max_idx * angle_increment

        # Converte il punto in coordinate cartesiane nel frame del robot
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        return x, y

    def transform_to_map(self, x, y):
        """
        Trasforma le coordinate dal frame del robot ('base_link') al frame della mappa ('map').
        """
        try:
            now = rospy.Time.now()
            self.tf_listener.waitForTransform('map', 'base_link', now, rospy.Duration(4.0))
            point = PoseStamped()
            point.header.frame_id = 'base_link'
            point.header.stamp = now
            point.pose.position.x = x
            point.pose.position.y = y
            point.pose.orientation.w = 1.0

            # Trasforma nel frame della mappa
            map_point = self.tf_listener.transformPose('map', point)
            return map_point.pose.position.x, map_point.pose.position.y
        except tf.Exception as e:
            rospy.logwarn(f"TF Transform failed: {e}")
            return None, None

    def publish_goal(self, x, y):
        """
        Pubblica un goal su /exploration_goal.
        """
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        rospy.loginfo(f"Published goal: ({x}, {y})")

    def explore(self):
        """
        Ciclo principale di esplorazione.
        """
        while not rospy.is_shutdown():
            rospy.loginfo("Analyzing LiDAR data for exploration...")
            best_goal = self.find_best_goal()

            if best_goal is None:
                rospy.loginfo("No valid goals found. Rotating...")
                self.rotate_robot()
                continue

            # Trasforma il goal nel frame della mappa
            map_x, map_y = self.transform_to_map(best_goal[0], best_goal[1])
            if map_x is not None and map_y is not None:
                self.publish_goal(map_x, map_y)

            rospy.sleep(2)  # Aspetta che il robot raggiunga il goal prima di analizzare di nuovo

    def rotate_robot(self):
        """
        Rotazione semplice per scansionare l'ambiente.
        """
        rospy.loginfo("Rotating robot to find new areas...")
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.orientation.z = 1.0  # Rotazione sul posto
        goal.pose.orientation.w = 0.0
        self.goal_pub.publish(goal)
        rospy.sleep(5)  # Aspetta qualche secondo per completare la rotazione

if __name__ == "__main__":
    explorer = ExplorationNode()
    explorer.explore()























#     def __init__(self):
#         rospy.init_node('exploration_node', anonymous=True)

#         # Publisher to send goals
#         self.goal_pub = rospy.Publisher('/exploration_goal', PoseStamped, queue_size=10)

#         # Subscriber to get laser scan data
#         rospy.Subscriber('/scan_raw', LaserScan, self.laser_callback)

#         # Parameters
#         self.safe_distance = 0.5

#     def laser_callback(self, msg):
#         """Process laser data and publish a goal to explore."""
#         # Find indices of scan ranges that are safe
#         safe_indices = [i for i, distance in enumerate(msg.ranges)
#                         if distance > self.safe_distance and not math.isinf(distance)]

#         if not safe_indices:
#             rospy.loginfo("No safe points detected.")
#             return

#         # Select a point to navigate to
#         chosen_index = np.random.choice(safe_indices)
#         angle = msg.angle_min + chosen_index * msg.angle_increment
#         distance = msg.ranges[chosen_index]

#         # Compute coordinates
#         x = distance * math.cos(angle)
#         y = distance * math.sin(angle)

#         # Create and publish the goal
#         goal = PoseStamped()
#         goal.header.frame_id = msg.header.frame_id  # Use the same frame as the laser scan
#         goal.header.stamp = rospy.Time.now()
#         goal.pose.position.x = x
#         goal.pose.position.y = y
#         goal.pose.orientation.w = 1.0  # Facing forward

#         self.goal_pub.publish(goal)
#         rospy.loginfo(f"Published goal to x: {x:.2f}, y: {y:.2f}")

# if __name__ == '__main__':
#     try:
#         node = ExplorationNode()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass