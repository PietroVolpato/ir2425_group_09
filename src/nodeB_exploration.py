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
        rospy.init_node('Exploration_node')
        rospy.loginfo("Exploration Node initialized.")

        # Sottoscrizione al topic del LiDAR
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Publisher per i goal
        self.goal_pub = rospy.Publisher('/exploration_goal', PoseStamped, queue_size=10)
        self.command_sub = rospy.Subscriber('/exploration_command', String, self.command_callback)
        self.status_pub = rospy.Publisher('/node_b_feedback', String, queue_size=10)

        # Sottoscrizione ai topic di status e feedback
        # self.status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)
        
        # Publisher per il comando di velocità
        self.cmd_vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
        # self.feedback_sub = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.feedback_callback)

        # TF listener per trasformazioni
        self.tf_listener = tf.TransformListener()

        # Variabili per i dati del LiDAR e lo stato del robot
        self.scan_data = None
        self.current_status = None
        self.min_distance_goal = 0.5  # Distanza minima per considerare valido un goal
        self.previous_goal = []  # Per evitare goal ridondanti
        self.command = None

    def scan_callback(self, msg):
        self.scan_data = msg

    def command_callback(self, msg):
        self.command = str(msg.data).lower()
        rospy.loginfo(f"Received command: {self.command})")
        if self.command == 'continue':
            self.publish_goal(x=7, y=0)
            self.explore()
        elif self.command == 'stop':
            rospy.loginfo("Stopping exploration...")

    def find_best_goal(self):
        """
        Analyze the laser data to find the best goal to move to.
        """
        if self.scan_data is None:
            rospy.logwarn("No LiDAR data received.")
            return None
        
        #Calculate the angle min and increment of the scan
        angle_min = self.scan_data.angle_min
        angle_increment = self.scan_data.angle_increment
        
        # Get the ranges of the LiDAR scan
        ranges = np.array(self.scan_data.ranges)
        
        # Find the maximum distance to move to
        max_distance = np.max(ranges)
        
        # Send the robot to the farthest point
        if max_distance > self.min_distance_goal:
            max_index = np.argmax(ranges)
            angle = angle_min + angle_increment * max_index
            x = max_distance * np.cos(angle)
            y = max_distance * np.sin(angle)
            return x, y

    def transform_to_map(self, x, y):
        while not self.tf_listener.canTransform('map', self.scan_data.header.frame_id, rospy.Time(0)):
            rospy.logwarn("Waiting for TF...")
            rospy.Duration(1.0).sleep()
            
        try:
            # Create a PoseStamped message in the 'base_link' frame
            base_goal = PoseStamped()
            base_goal.header.frame_id = self.scan_data.header.frame_id
            base_goal.header.stamp = rospy.Time.now()
            base_goal.pose.position.x = x
            base_goal.pose.position.y = y
            base_goal.pose.orientation.w = 1.0

            # Transform the pose to the 'map' frame
            self.tf_listener.waitForTransform('map', base_goal.header.frame_id, rospy.Time(0), rospy.Duration(10.0))
            map_goal = self.tf_listener.transformPose('map', base_goal)

            # Extract transformed coordinates
            x_map = map_goal.pose.position.x
            y_map = map_goal.pose.position.y

            return x_map, y_map
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Failed to transform goal from " + base_goal.header.frame_id + " to map frame.")
            return None, None

    def publish_goal(self, x, y):
        """
        Publish goal to /exploration_goal.
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
       Wait until the goal is reached or failed.
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

    def explore(self):
        """"
        Explore the map to find the next postiion to move to.
        """
        self.status_pub.publish(String(data="Exploration started."))
        while not rospy.is_shutdown():
            # Find the best goal in the environment
            best_goal = self.find_best_goal()
            if best_goal is None:
                rospy.logwarn("No valid goals found.")
                break

            # Transform the goal to the map frame
            x, y = self.transform_to_map(*best_goal)
            if x is None or y is None:
                rospy.logwarn("Failed to transform goal.")
                break

            # Publish the goal
            self.publish_goal(x, y)

            # Wait for the goal result
            if not self.wait_for_goal_result():
                # Rotate the robot to find new areas
                self.rotate_360()

    def rotate_360(self):
        """
        This function is called once at the beginning, and is needed to let the camera see apriltags which are placed
        behind or aside of tiago.
        An appropriate angular velocity is applied to make tiago rotate on himself of 360°.
        """

        # Define the Twist message for rotation
        rotate_cmd = Twist()
        rotate_cmd.angular.z = 0.4  # Angular velocity in radians/second (counterclockwise)

        # Calculate rotation duration for 360 degrees
        rotation_duration = 2 * 3.14159 / abs(rotate_cmd.angular.z) 

        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.Time.now()

        self.status_pub.publish(String(data="Performing 360° rotation."))  # update node_A about the new tag found.status_pub
        while (rospy.Time.now() - start_time).to_sec() < rotation_duration:
            self.cmd_vel_pub.publish(rotate_cmd)
            rate.sleep()
        self.status_pub.publish(String(data="Terminated 360° rotation."))
        # Stop the robot after rotation
        self.cmd_vel_pub.publish(Twist())  # Publish zero velocity
        rospy.sleep(1)

    def run(self):
        rospy.spin()
    
if __name__ == "__main__":
    explorer = ExplorationNode()
    explorer.run()