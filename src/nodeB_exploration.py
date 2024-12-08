#!/usr/bin/env python3

import rospy
import math
import random
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs

class ExplorationNode:
    def __init__(self):
        rospy.init_node('Exploration_node')
        rospy.loginfo("Exploration Node initialized.")

        # Sottoscrizione al topic del LiDAR
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Subscriber per i comandi di velocità
        self.cmd_vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

        # Publisher per i goal
        self.goal_pub = rospy.Publisher('/exploration_goal', PoseStamped, queue_size=10)
        self.command_sub = rospy.Subscriber('/exploration_command', String, self.command_callback)
        self.status_pub = rospy.Publisher('/node_b_feedback', String, queue_size=10)

        # TF listener per trasformazioni
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Sottoscrizione ai topic della mappa e del costmap
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.costmap_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.costmap_callback)

        # Variabili per i dati del LiDAR e lo stato del robot
        self.scan_data = None
        self.command = None
        self.occupancy_grid = None
        self.costmap = None
        self.frontiers = []
        self.visited_points = []
        self.visited_threshold = 0.5

    def scan_callback(self, msg):
        self.scan_data = msg

    def command_callback(self, msg):
        self.command = str(msg.data).lower()
        rospy.loginfo(f"Received command: {self.command}")
        if self.command == 'continue':
            self.explore()
        elif self.command == 'stop':
            rospy.loginfo("Stopping exploration...")

    def map_callback(self, msg):
        self.occupancy_grid = msg
        rospy.loginfo("Occupancy grid data received.")

    def costmap_callback(self, msg):
        self.costmap = msg
        
    def detect_frontiers(self):
        """
        Detect frontiers based on LiDAR data.
        """
        if self.scan_data is None:
            return []

        frontiers = []
        angle_min = self.scan_data.angle_min
        angle_increment = self.scan_data.angle_increment
        ranges = np.array(self.scan_data.ranges)


        for i, distance in enumerate(ranges):
            angle = angle_min + i*angle_increment
            frontier_x = distance * np.cos(angle)
            frontier_y = distance * np.sin(angle)

            map_x, map_y = self.transform_to_map(frontier_x, frontier_y)
            if map_x is not None and map_y is not None:
                frontiers.append((map_x, map_y))

        return frontiers
    
    def is_visited(self, point):
        """
        Check if a point has already been visited.
        """
        for visited in self.visited_points:
            if np.linalg.norm(np.array(point) - np.array(visited)) < self.visited_threshold:
                return True
        return False
    
    def add_visited_point(self, point):
        """
        Add a point to the list of visited points.
        """
        self.visited_points.append(point)

    def assign_probabilities(self, weights):
        """
        Assign a probability distribution to the frontiers.
        """
        if not self.frontiers:
            return [], []

        robot_x, robot_y = self.get_robot_position()
        distance_robot = []
        distance_visited = []
        probabilities = []
        for f in self.frontiers:
            # Distance from robot
            distance_robot.append(np.linalg.norm(np.array(f) - np.array((robot_x, robot_y))))
            
            # Distance from visited points
            if self.visited_points:
                distance_visited.append(min([np.linalg.norm(np.array(f) - np.array(v)) for v in self.visited_points]))
            else:
                distance_visited.append(0)

        # Normalize distances
        max_distance_robot = max(distance_robot) if distance_robot else 1.0
        max_distance_visited = max(distance_visited) if distance_visited else 1.0

        distance_robot_norm = [d / (max_distance_robot + 1e-6) for d in distance_robot]
        distance_visited_norm = [d / (max_distance_visited + 1e-6) for d in distance_visited]

        # Combine the two distances to get the probability distribution
        for dr, dv in zip(distance_robot_norm, distance_visited_norm):
            score = weights[0] * dr + weights[1] * dv
            probabilities.append(score)

        # Normalize probabilities
        total = sum(probabilities)
        probabilities = [p / total for p in probabilities] if total > 0 else [1.0 / len(probabilities)] * len(probabilities)
              
        return probabilities

    def sample_goal(self, frontiers, probabilities):
        """
        Sample a goal based on the frontiers and their probabilities.
        """
        if not frontiers or not probabilities:
            return None

        # Take the goal that has the highest probability
        goal_index = np.argmax(probabilities)

        return frontiers[goal_index]
    
    def transform_to_map(self, x, y):
        try:
            # Create PoseStamped in LiDAR frame
            base_goal = PoseStamped()
            base_goal.header.frame_id = self.scan_data.header.frame_id
            base_goal.header.stamp = rospy.Time.now()
            base_goal.pose.position.x = x
            base_goal.pose.position.y = y
            base_goal.pose.orientation.w = 1.0

            # Look up transform
            transform = self.tf_buffer.lookup_transform('map', base_goal.header.frame_id, rospy.Time(0), rospy.Duration(10.0))

            # Transform pose
            map_goal = tf2_geometry_msgs.do_transform_pose(base_goal, transform)

            x_map = map_goal.pose.position.x
            y_map = map_goal.pose.position.y

            return x_map, y_map
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to transform goal from {base_goal.header.frame_id} to 'map': {e}")
            return None, None
        
    def get_robot_position(self):
        try:
            # Look up transform
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(10.0))

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            return x, y
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to get robot position: {e}")
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
        goal.pose.orientation.z = 0.0
        self.goal_pub.publish(goal)
        rospy.loginfo(f"Published goal: ({x}, {y}, {goal.pose.orientation.w}, {goal.pose.orientation.z})")

    def explore(self):
        """"
        Explore the map to find the next postiion to move to.
        """

        self.frontiers = self.detect_frontiers()
        probabilities = self.assign_probabilities((0.6, 0.4))
        # rospy.loginfo(f"Frontiers: {frontiers}")
        # rospy.loginfo(f"Probabilities: {probabilities}")

        goal = self.sample_goal(self.frontiers, probabilities)
        if goal is None:
            rospy.logwarn("No valid frontier found.")
            return
        
        goal = self.adjust_goal_position(*goal)
        
        self.add_visited_point(goal)
        self.publish_goal(goal[0], goal[1])
        self.frontiers.clear()

    def is_free_space(self, x, y):
        if self.occupancy_grid is None:
            rospy.logwarn("Occupancy grid not yet initialized. Cannot check free space.")
            return False

        resolution = self.occupancy_grid.info.resolution
        origin_x = self.occupancy_grid.info.origin.position.x
        origin_y = self.occupancy_grid.info.origin.position.y

        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)

        width = self.occupancy_grid.info.width
        height = self.occupancy_grid.info.height

        # Check bounds
        if 0 <= grid_x < width and 0 <= grid_y < height:
            index = grid_y * width + grid_x
            return self.occupancy_grid.data[index] == 0  # 0 indicates free space
        else:
            rospy.logwarn("Point out of bounds in occupancy grid.")
            return False

    def adjust_goal_position(self, x, y):
        # Adjust the goal position to a nearby free space

        for dx in range(-2, 3):
            for dy in range(-2, 3):
                new_x = x + dx
                new_y = y + dy
                if self.is_free_space(new_x, new_y):
                    return new_x, new_y
        return x, y  # Return original position if no free space found
        return x, y  # Return original position if no free space found
    
    
    def run(self):
        rospy.spin()
    
if __name__ == "__main__":
    explorer = ExplorationNode()
    explorer.run()
