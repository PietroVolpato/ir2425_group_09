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
from collections import deque

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

        # publisher to say that the navigation passed to control law mode
        self.control_law_pub = rospy.Publisher('/control_law_command', String, queue_size=10)

        # subscriber to get feedback if a goal was successful, in order to add it to the queue
        self.goal_feedback_sub = rospy.Subscriber('/navigation_feedback', String, self.queue_goal_callback)

        # TF listener per trasformazioni
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Sottoscrizione ai topic della mappa e del costmap
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Variabili per i dati del LiDAR e lo stato del robot
        self.scan_data = None      
        self.occupancy_grid = None
        self.previous_goals = deque([(0, 0)], maxlen=5)  # queue containing 5 previous goals start considering (0, 0) as a visited goal.

        # Control parameters for motion control law navigation
        self.min_distance_obstacle_threshold = 0.3  # the control recognize as in front of an obscacle if central lidar ranges are below
        self.min_distance_wall_threshold = 0.4  # the control law perform recognize as close a wall if is below this distances.
        self.side_clearance = 1  # threshold for activating the control law looking distances on left and right side
        self.linear_speed = 0.45  # Base forward speed (m/s) for control law. This number will be normalized (always lowered) based on the detected distances
        self.angular_speed = 0.2  # Base turning speed (rad/s) for control law. This number will be normalized based on the detected distances
        self.chosen_side = None  #  variable used to handle the case tiago is in front of an obstacle. Once is fixed, we stick to that side to avoid obstacle.
        self.control_law_active = False # keep track is the control mode is active

        self.exploration_active = False # exploration starts with first continue coommand, and ends with stop command
        self.current_goal = None

    def scan_callback(self, msg):
        self.scan_data = msg
        self.control_law()  # Added call to control law

    def queue_goal_callback(self, msg):
        """
        When a goal is reached, we append it on the left of the queue of previous goals.
        A String message is also sent to node A, containing the history of goals.
        """
        feedback = str(msg.data).lower()
        s = ""
        if feedback == "goal reached":
            self.previous_goals.appendleft(self.current_goal) # append tuple to previous goals
            for elem in self.previous_goals:
                x = "{:.3f}".format(elem[0])
                y = "{:.3f}".format(elem[1])
                s += f"({x},{y}) "
            self.status_pub.publish(String(data=f"QUEUE OF PREVIOUS GOALS: {s}"))

    def command_callback(self, msg):
        command = str(msg.data).lower()
        rospy.loginfo(f"Received command: {command}")
        if command == 'continue':
            self.exploration_active = True
            self.explore()
        elif command == 'stop':
            rospy.loginfo("Stopping exploration...")
            self.exploration_active = False
        else:
            rospy.logerr("Unknown command. Stopping exploration.")
    def map_callback(self, msg):
        self.occupancy_grid = msg
        rospy.loginfo("Occupancy grid data received.")
    
    def random_sample(self, sector, angle_min, angle_increment):
        """
        Generates a random goal from a sector of lidar data.
        """
        min_free_space = 2  # Minimum required distance from obstacles for picking a sample
        safe_distance = 0.8  # send tiago far enough from osbacle
        max_attempts = 10  # Maximum number of attempts to try find a valid sample

        if not sector:
            return None

        attempts = 0
        while attempts < max_attempts:
            # pick a random sample for the considered sector of lidar data
            random_index = random.randint(0, len(sector) - 1)
            distance = sector[random_index][0]
            position = sector[random_index][1]

            #check there is enough room
            if distance < min_free_space:
                attempts += 1
                continue

            corrected_distance = distance - safe_distance
            angle = angle_min + position * angle_increment

            x = corrected_distance * np.cos(angle)
            y = corrected_distance * np.sin(angle)
            x_map, y_map = self.transform_to_map(x, y)

            # check if sample is suitable
            d = 0.3  # distance defined for square neighborhood

            neighborhood_free =  self.is_free_space(x_map+d, y_map+d) and self.is_free_space(x_map+d, y_map-d) and self.is_free_space(x_map-d, y_map+d) and self.is_free_space(x_map-d, y_map-d)
            if self.is_free_space(x_map, y_map) and neighborhood_free:
                return (x_map, y_map, corrected_distance)

            attempts += 1

        return None  # Failed to find a suitable sample

    def explore(self):
        """
        Processes lidar data to find the next exploration goal.
        """
        #parameters
        distance_power = 1.5 # defines how much we care of reaching goals far from visited goals
        number_of_sectors_sampled = 8  # defines how many samples we take from lidar data, one per circular sector

        self.status_pub.publish(String(data="Processing LIDAR DATA to find goal"))
        angle_min = self.scan_data.angle_min
        angle_increment = self.scan_data.angle_increment
        ranges = np.array(self.scan_data.ranges)

        # Create dataset as a list of (distance, index) tuples
        dataset = [(r, i) for i, r in enumerate(ranges)]
        
        sectors = []

        # Split dataset into sectors
        n = len(dataset)
        interval = n // number_of_sectors_sampled
        for i in range(number_of_sectors_sampled):
            if i == number_of_sectors_sampled - 1:  # Last sector
                sectors.append(dataset[i * interval :])
            else:
                sectors.append(dataset[i * interval : (i + 1) * interval])

        # Sample a point from each sector
        samples = []
        for sector in sectors:
            sample = self.random_sample(sector, angle_min, angle_increment)
            if sample is not None:
                samples.append(sample)
         

        # if there is no valid points to go, we set the trivial goal (current position, then thanks to the random rotation we will find some goal)
        if not samples:
            rospy.logwarn("No valid goal found in lidar data. Setting fallback goal.")
            # Fallback goal: Tiago's current position
            current_position = self.get_current_position()
            self.publish_goal(current_position[0], current_position[1])
            return

        average_distances = np.array([(self.average_distance_from_previous_goals(x, y))**distance_power for x, y, r in samples])

        # Stabilize the softmax computation
        max_value = np.max(average_distances)
        # Compute probability distribution based on average distance from previous goals
        exp_values = np.exp(average_distances - max_value)
        # softmax probability distribution, based on distances of the samples. 
        probability_distribution = exp_values / np.sum(exp_values)

        # Choose a sample based on the probability distribution
        selected_index = np.random.choice(len(samples), p=probability_distribution)
        chosen_sample = samples[selected_index]

        # Publish the chosen sample as the next goal
        self.current_goal = (chosen_sample[0], chosen_sample[1])
        self.publish_goal(chosen_sample[0], chosen_sample[1])

    def average_distance_from_previous_goals(self, x, y):
        """
        Computes the average distance from the given point to all previous goals.
        """
        if not self.previous_goals:
            return 0  # If no previous goals, treat distance as zero leading to uniform probability distribution

        avg_distance = 0
        for x_goal, y_goal in self.previous_goals:
            avg_distance += math.sqrt((x - x_goal)**2 + (y - y_goal)**2)
        return avg_distance / len(self.previous_goals)

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
        
    def get_current_position(self):
        """
        Retrieves Tiago's current position from TF.
        """
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0))
            position = transform.transform.translation
            return position.x, position.y
        except Exception as e:
            rospy.logerr(f"Failed to get current position: {e}")
            # If current position cannot be determined, fall back to (0, 0)
            return 0.0, 0.0
    
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
    def control_law(self):
        """
        
        
        """
        if self.scan_data is None or not self.exploration_active:  # not ready to start exploration, or task finished, or problem with lidar data
            return

        # Extract distances to obstacles
        ranges = np.array(self.scan_data.ranges)
        n = len(ranges)
        
        # Filter out invalid or extreme range values
        ranges = np.clip(ranges, 0, 10)  # Assume max range of 10 meters

        angle_sides = math.pi/6  # 30 degrees
        side_samples =  int(angle_sides/self.scan_data.angle_increment)

        angle_front = math.pi/9  # 20 degrees
        front_samples =  int(angle_front/self.scan_data.angle_increment)

        # Get distances to the left, front, and right of the robot
        left_indices = np.arange(n - side_samples, n) % n
        front_indices = np.arange(int(n/2 - front_samples/2), int(n/2 + front_samples/2)) % n
        right_indices = np.arange(0, side_samples) % n

        left_distance = np.mean(ranges[left_indices])
        front_distance = np.mean(ranges[front_indices])
        right_distance = np.mean(ranges[right_indices])

        rospy.loginfo(f"Distances - Left: {left_distance:.2f} m, Front: {front_distance:.2f} m, Right: {right_distance:.2f} m")

        twist = Twist()
        # **Decision to enter corridor control mode: control law already active or few room on left and right.
        # the exit of the control law is ruled by hysterisis thresholding, to avoid unstable enter/exit of the mode
        if self.control_law_active or (left_distance < self.side_clearance and right_distance < self.side_clearance):
            
            if not self.control_law_active: # give information only when starting the control law
                self.status_pub.publish(String(data=f"Narrow passage detected, ACTIVATING CONTROL LAW"))  # feedback to node A
                rospy.loginfo("Narrow passage detected, activating control law.")
                self.control_law_pub.publish(String(data="control law on"))  # inform the navigation, will cancel current goal.
                self.control_law_active = True

            sign = -1 if right_distance > left_distance else 1  # define clockwise or counterclockwise rotation

            # OBSTACLE MODE. too close to an obstacle in front
            if front_distance < self.min_distance_obstacle_threshold:
                print("OBSTACLE")
                if not self.chosen_side:  # chose one side to take to avoid the obstacle
                    self.chosen_side = "right" if right_distance > left_distance else "left"
                # positive if right > left, negative otherwise. rotate looking a direction with more room to move
                twist.angular.z = -self.angular_speed if self.chosen_side == "right" else self.angular_speed
                # don't move forward until there is no obstacle in front anymore
                twist.linear.x = 0

            # WALL MODE. got close to a wall on the sides
            elif  left_distance < self.min_distance_wall_threshold or right_distance < self.min_distance_wall_threshold:
                print("WALL")
                # positive if right > left, negative otherwise. rotate looking a direction with more room to move
                twist.angular.z = sign * self.angular_speed
                # signlificantly reduce speed to avoid collision.
                twist.linear.x = self.linear_speed * min(left_distance, right_distance) / self.side_clearance
                self.chosen_side = None  # there is no obstacle ahead, reset chosen_side
            else:
                # CLEAR MODE
                print("CLEAR")
                # Move forward
                twist.linear.x = self.linear_speed
                # slightly rotate to try keep the center of the corridor
                twist.angular.z = sign * self.angular_speed / 2
                self.chosen_side = None  # there is no obstacle ahead, reset chosen_side
            rospy.loginfo(f"Publishing Twist - Linear: {twist.linear.x:.2f}, Angular: {twist.angular.z:.2f}")

            self.cmd_vel_pub.publish(twist)    # publish the velocity command
        # hysterisis thresholding to exit the control law
        if self.control_law_active and (left_distance >= (self.side_clearance + 0.3) or right_distance >= (self.side_clearance + 0.3)):
            self.status_pub.publish(String(data=f"Narrow passage passed, DISABLING CONTROL LAW"))  # feedback to node A
            self.control_law_pub.publish(String(data="control law off"))
            self.control_law_active = False
    
if __name__ == "__main__":
    explorer = ExplorationNode()
    rospy.spin()
