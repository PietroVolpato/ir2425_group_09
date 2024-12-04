#!/usr/bin/env python3

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionFeedback
from std_msgs.msg import String
import tf

class ExplorationNode:
    def __init__(self):
        rospy.init_node('simple_exploration_node')

        # Sottoscrizione al topic del LiDAR
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Publisher per i goal
        self.goal_pub = rospy.Publisher('/exploration_goal', PoseStamped, queue_size=10)
        self.command_sub = rospy.Subscriber('/exploration_command', String, self.command_callback)

        # Sottoscrizione ai topic di status e feedback
        self.status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)
        
        # Publisher per il comando di velocità
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.feedback_sub = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.feedback_callback)

        # TF listener per trasformazioni
        self.tf_listener = tf.TransformListener()

        # Variabili per i dati del LiDAR e lo stato del robot
        self.scan_data = None
        self.current_status = None
        self.last_feedback_position = None
        self.min_distance_goal = 0.5  # Distanza minima per considerare valido un goal
        self.previous_goal = None  # Per evitare goal ridondanti
        self.initial_fallback_done = False
        self.command = None

    def scan_callback(self, msg):
        self.scan_data = msg

    def status_callback(self, msg):
        if msg.status_list:
            # Prendi lo stato dell'ultimo goal (il più recente)
            self.current_status = msg.status_list[-1].status
            if self.current_status == 1:
                rospy.loginfo("Goal is active (being processed).")
            elif self.current_status == 3:
                rospy.loginfo("Goal succeeded!")
            elif self.current_status == 4:
                rospy.logwarn("Goal was aborted.")
            elif self.current_status == 5:
                rospy.logwarn("Goal was rejected.")
            elif self.current_status == 9:
                rospy.loginfo("Goal was cancelled.")

    def feedback_callback(self, msg):
        # Salva la posizione corrente dal feedback
        self.last_feedback_position = msg.feedback.base_position.pose
        rospy.loginfo(f"Current robot position: x={self.last_feedback_position.position.x}, y={self.last_feedback_position.position.y}")

    def command_callback(self, msg):
        self.command = msg.data.lower()
        rospy.loginfo(f"Received command: {self.command})")
        if self.command == 'continue':
            self.explore()
        elif self.command == 'stop':
            rospy.loginfo("Stopping exploration...")

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
        valid_indices = np.where((ranges > self.min_distance_goal) & (ranges < self.scan_data.range_max))[0]
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

    def wait_for_goal_result(self):
        """
        Attende finché il goal non viene completato o interrotto.
        """
        rospy.loginfo("Waiting for goal result...")
        while not rospy.is_shutdown():
            if self.current_status == 3:  # SUCCEEDED
                rospy.loginfo("Goal reached successfully!")
                return True
            elif self.current_status in [4, 5, 9]:  # ABORTED, REJECTED, CANCELLED
                rospy.logwarn("Goal failed or cancelled.")
                return False
            rospy.sleep(0.5)

    def perform_initial_fallback(self):
        """
        Movimento iniziale per uscire da situazioni statiche.
        """
        rospy.loginfo("Performing initial fallback motion...")
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = 7
        goal.pose.position.y = 0
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.initial_fallback_done = True

    def explore(self):
        """
        Ciclo principale di esplorazione.
        """
        while not rospy.is_shutdown():
            # if not self.initial_fallback_done:
            #     self.perform_initial_fallback()
            #     continue
            rospy.loginfo("Analyzing LiDAR data for exploration...")
            best_goal = self.find_best_goal()

            if best_goal is None:
                rospy.loginfo("No valid goals found. Rotating...")
                self.rotate_robot()
                continue

            # Trasforma il goal nel frame della mappa
            map_x, map_y = self.transform_to_map(best_goal[0], best_goal[1])
            if map_x is not None and map_y is not None:
                # Controlla se il goal è lo stesso dell'ultimo per evitare ridondanza
                if self.previous_goal and math.isclose(map_x, self.previous_goal[0], abs_tol=0.1) and math.isclose(map_y, self.previous_goal[1], abs_tol=0.1):
                    rospy.loginfo("Goal is too close to the previous one, skipping...")
                    continue

                self.previous_goal = (map_x, map_y)
                self.publish_goal(map_x, map_y)

                # Attendi il risultato del goal
                if not self.wait_for_goal_result():
                    rospy.logwarn("Moving to next goal due to failure.")

    def rotate_robot(self):
        """
        Rotazione semplice per scansionare l'ambiente.
        """
        rospy.loginfo("Rotating robot to find new areas...")
        # Ruota sul posto pubblicando un goal temporaneo
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.orientation.z = 0.707  # 90 gradi in quaternion
        goal.pose.orientation.w = 0.707
        self.goal_pub.publish(goal)
        rospy.sleep(5)

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