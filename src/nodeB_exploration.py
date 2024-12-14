#!/usr/bin/env python3
"""
Exploration Node
...
IMPORTANT NOTE: in the particular map given, it is easy to see that, after tiago rotatesof 360 and pass the corridor all the tags on the 
right of the beginning of the corridor have always been detected. Hence if would be good, in this case, to force a goal to be in the core room,
and never entering the corridor. However, in the general case, there is no grant that we explored the room on the other side of the corridor,
so it would be a terrible mistake to forbid the robot to reach that room again, and potentially making the task always fail.
so, by implementation choice WE DECIDED to stick to the general case: hence to allow the algoritm to generate a goal leading to the corridor 
(again) and by consequence activating the controw and goingh trowugh it. Tiago will then explore the small room at the beginning 
(even if there are never new tags), and in a small time go back through the corridor (control law) and get back to the core room.
We though that is better to don't base the algorthm on the characteristic of this particular map, when possible.
"""
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
        self.side_clearance = 0.7  # threshold for activating the control law looking distances on left and right side
        self.front_clearence = 1.5 # # threshold for activating the control law: a narrow passage is detected if tiago has some room in front.
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
        """
        Syncronization with the goal management node:
        if a command "continue" is received, a goal is generated and sent to the navigation node
        if the command "stop" is received, then it means task is completed an this node is not producing goals anymore.
        """
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
    
    def table_assumption_check(self, r, angle, increment = 0.2):
        """
        We assume the table to be static, and this function checks that the path to a candidate goal does
        not intersect with the region covered  by the table (lidar can't see it)
        r : distance from tiago and the candidate goal
        angle: angle (polar coordinates) extracted by lidar data
        increment: interval of sampling, to check if a point in the line is intersecting the table region

        return: true if by sampling the range we don't intersect the table region, false otherwise
        """
        # coordinates of the table hard coded (actually a square a little bit larger)
        x1 = 6 #6.3 # x1 < x2 must be satisfied
        x2 = 8.4 #8.1
        y1 = -3.6 #-3.3 # y1 < y2 must be satisfied
        y2 = -1.5 #-1.8
        for i in np.arange(increment, r, increment):
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            x_map, y_map = self.transform_to_map(x, y)
            if x1 <= x_map <= x2 and y1 <= y_map <= y2:  # sample point is inside the table
                return False
        return True
    
    def neighborhood_check(self, r, angle, neighborhood_size = 0.4, increment = 0.22):
        """
        r : distance from tiago and the candidate goal
        angle: angle (polar coordinates) extracted by lidar data
        neighborhood_size : size of the square neighborhood to check. i.e. a square 2*neighborhood_size x 2*neighborhood_size centered on the current sample
        increment : sampling interval on the straight line from tiago to goal

        This function checks the final part of the trajectory from tiago to goal, ensuring that there is ehough room for tiago to pass and reach goal.
        For every point sampled, 16 points on the square centered on it are checked to be free.

        return : True if in the neighborhood were sampled only free points, False otherwise
        """

        check_distance = 1 # checks the last part of the trajectory [meters]
        # iterate on few points in the last part of the trajectory
        for r_sample in np.arange(r-check_distance, r, increment):
                # compute coordinates of the sampled point in the trajectory
                x = r_sample * np.cos(angle)
                y = r_sample * np.sin(angle)
                x, y = self.transform_to_map(x, y)

                d = neighborhood_size # just to have a short name

                # check 16 points belonging to square neighborhood
                c1 =  self.is_free_space(x+d,y+d) and self.is_free_space(x+d,y-d) and self.is_free_space(x-d,y+d) and self.is_free_space(x-d,y-d)
                c2 =  self.is_free_space(x,y+d) and self.is_free_space(x,y-d) and self.is_free_space(x+d,y) and self.is_free_space(x-d,y)
                c3 =  self.is_free_space(x+d/2,y+d) and self.is_free_space(x+d/2,y-d) and self.is_free_space(x-d/2,y+d) and self.is_free_space(x-d/2,y-d)
                c4 =  self.is_free_space(x+d,y+d/2) and self.is_free_space(x-d,y+d/2) and self.is_free_space(x+d,y-d/2) and self.is_free_space(x-d,y-d/2)

                if c1 and c2 and c3 and c4:  # inner square free
                    continue
                else:  # point very close which is not free
                    return False
        return True


    def random_sample(self, sector, angle_min, angle_increment):
        """
        sector: a portion of lidar data, covering a certain circular sector. Lidar data are formatted in an array of tupes, respectivevly
                the measured range and the relative original index
        angle_min: the angle min of the lidar
        angle_increment: the angle increment of the lidar

        This function picks an element of the sector at random, and check if satisfy requirements to be a valid goal: presence of safe distance,
        goal trajectory must not intersect with table region, a small neighborhood of the goal must be "free space".
        If the requirements for a sample are satisfied, it is redurned as a valid possible goal, otherwise we keep trying to find a valid sample
        in the sector. After max_attempts, we give up on returning a sample of the sector.

        return: (x_map, y_map) respectively x and y of the goal w.r.t. map reference frame.
        """
        min_free_space = 2  # Minimum required distance from obstacles for picking a sample
        safe_distance = 1  # send tiago far enough from osbacle
        max_attempts = 10  # Maximum number of attempts to try find a valid sample

        if not sector:
            return None

        attempts = 0
        while attempts < max_attempts:
            # pick a random sample for the considered sector of lidar data
            random_index = random.randint(0, len(sector) - 1)
            distance = sector[random_index][0]
            position = sector[random_index][1]

            corrected_distance = distance - safe_distance
            angle = angle_min + position * angle_increment

            # check there is enough room or the line intersect table region
            if distance < min_free_space or not self.table_assumption_check(corrected_distance, angle, increment=0.3):
                attempts += 1
                continue

            x = corrected_distance * np.cos(angle)
            y = corrected_distance * np.sin(angle)
            x_map, y_map = self.transform_to_map(x, y)

            # check if candidate goal has a "close" neighborhood all free
            if self.neighborhood_check(corrected_distance, angle):
                return (x_map, y_map)

            attempts += 1

        return None  # Failed to find a suitable sample

    def explore(self):
        """
        Processes lidar data to find the next exploration goal.
        The whole array of lidar data is splitted in number_of_sectors_sampled circular sectors.
        It is tried to extract a valid sample (candidate goal) per sector. See random_sample()
        Once we collected all candidate goals, we compute the average distance between each candidate goal and all
        previous goals (in the queue self.previous goals).
        The average distances are elevated to the distance_power, defining the importance of the distance.
        Then the array of distances is converted into a probability distribution using a softmax function,
        And then the final goal is chosen at random from the candidate goals, according to such distribution.
        """
        #parameters
        distance_power = 1.5 # defines how much we care of reaching goals far from visited goals
        number_of_sectors_sampled = 20  # defines how many samples we take from lidar data, one per circular sector

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

        average_distances = np.array([(self.average_distance_from_previous_goals(x, y))**distance_power for x, y in samples])

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
        """
        Transofrm the given x,y represented in tiago reference frame, into the map reference frame
        """
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
            return (position.x, position.y)
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

        # Neutral orientation (no specific heading)
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)
        rospy.loginfo(f"Exploration: published goal: ({x}, {y}, {goal.pose.orientation.w}, {goal.pose.orientation.z})")


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
        This is the control law logic, which is applied if a narrow corridor is detected.
        From the array of lidar measures, we extract right and left samples, which are all the measurements starting from the two
        extrema of the lidar covering angle_sides radians. We also extract the front samples, which are the central measurements covering 
        angle_front radians.
        Then if there is few distance on the sides (see side_clearance) and enough room in front (see front_clearence), the control law becomes active.
        There are three modes, listed in order of priority (e.g. mode 3 can't be active if mode 1 or mode 2 conditions are true):
        1) OSTACLE MODE. Active if there is few room in front. To avoid collision linear speed is set to 0 and a proper rotation is performed.
        2) WALL MODE: the distance from the side is low. To avoid collision linear speed is reduced, and a proper rotation is applied to get
            further from the wall
        3) CLEAR MODE: there is enough room both in front and on the sides. The linear speed is kept relatively high, and a small rotation is applied in
            order to try to keep the center of the corridor.
        
        To exit the control law we require that there is a good amount of room on the sides (no more narrow corridor).
        It is apllied hysterisis thresholding (stricter requirement to exit than to enter control law), in order to avoid consecutive ON/OFF of the control
        law.   
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

        #rospy.loginfo(f"Distances - Left: {left_distance:.2f} m, Front: {front_distance:.2f} m, Right: {right_distance:.2f} m")

        twist = Twist()

        # Decision to enter corridor control mode: control law already active or few room on left and right, and enough room in front.
        if self.control_law_active or (left_distance < self.side_clearance and right_distance < self.side_clearance and front_distance > self.front_clearence):
            
            if not self.control_law_active: # give information only when starting the control law
                self.status_pub.publish(String(data=f"Narrow passage detected, ACTIVATING CONTROL LAW"))  # feedback to node A
                rospy.loginfo("Narrow passage detected, activating control law.")
                self.control_law_pub.publish(String(data="control law on"))  # inform the navigation, will cancel current goal.
                self.control_law_active = True

            sign = -1 if right_distance > left_distance else 1  # define clockwise or counterclockwise rotation

            # OBSTACLE MODE. too close to an obstacle in front
            if front_distance < self.min_distance_obstacle_threshold:
                if not self.chosen_side:  # chose one side to take to avoid the obstacle
                    self.chosen_side = "right" if right_distance > left_distance else "left"
                # positive if right > left, negative otherwise. rotate looking a direction with more room to move
                twist.angular.z = -self.angular_speed if self.chosen_side == "right" else self.angular_speed
                # don't move forward until there is no obstacle in front anymore
                twist.linear.x = 0

            # WALL MODE. got close to a wall on the sides
            elif  left_distance < self.min_distance_wall_threshold or right_distance < self.min_distance_wall_threshold:
                # positive if right > left, negative otherwise. rotate looking a direction with more room to move
                twist.angular.z = sign * self.angular_speed
                # signlificantly reduce speed to avoid collision.
                twist.linear.x = self.linear_speed * min(left_distance, right_distance) / self.side_clearance
                self.chosen_side = None  # there is no obstacle ahead, reset chosen_side
            # CLEAR MODE
            else:               
                # Move forward
                twist.linear.x = self.linear_speed
                # slightly rotate to try keep the center of the corridor
                twist.angular.z = sign * self.angular_speed / 2
                self.chosen_side = None  # there is no obstacle ahead, reset chosen_side
            #rospy.loginfo(f"Publishing Twist - Linear: {twist.linear.x:.2f}, Angular: {twist.angular.z:.2f}")

            self.cmd_vel_pub.publish(twist)    # publish the velocity command
        # hysterisis thresholding to exit the control law
        if self.control_law_active and (left_distance >= (self.side_clearance + 0.3) or right_distance >= (self.side_clearance + 0.3)):
            self.status_pub.publish(String(data=f"Narrow passage passed, DISABLING CONTROL LAW"))  # feedback to node A
            self.control_law_pub.publish(String(data="control law off"))
            self.control_law_active = False
    
if __name__ == "__main__":
    explorer = ExplorationNode()
    rospy.spin()
