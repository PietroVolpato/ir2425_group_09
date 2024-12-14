#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
from tiago_iaslab_simulation.srv import Objs
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32MultiArray
import math
import random


class GoalManagementNode:
    def __init__(self):
        rospy.init_node('nodeB_goal_management', anonymous=True)

        # subscriber to get the target tags IDs from node A
        self.tag_sub = rospy.Subscriber('/target_ids', Int32MultiArray, self.get_targets_callback)

        # Subscriber to get the apriltags which are detected by the camera while Tiago is moving. From nodeB_apriltag_detection
        self.tag_sub = rospy.Subscriber('/detected_tags', PoseArray, self.tag_callback)

        # Subscriber to get the feedback about navigation (e.g. goal reached or aborted). From nodeB_navigation
        self.nav_feedback_sub = rospy.Subscriber('/navigation_feedback', String, self.navigation_callback)


        # Publisher to send commands for the nodeB_exploration ( e.g. "Continue", "Stop").
        self.exploration_command_pub = rospy.Publisher('/exploration_command', String, queue_size=10)

        # Publisher to send to node_A the apriltags positions, when all requested tags are detected
        self.final_results_pub = rospy.Publisher('/final_cube_positions', PoseArray, queue_size=10)

        # Publisher to send any feedback to node_A (e.g. "task completed")
        self.status_pub = rospy.Publisher('/node_b_feedback', String, queue_size=10)

        # publisher to send direct velocity commands
        self.cmd_vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)


        self.head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10) # to move the camera angle
  
        self.target_ids = []

        self.found_tags = {}  # disctionary to store positions of found tags
        self.exploration_active = False  # Indicates whether task is ongoing


    def rotation(self, angle, theta):
        """
        This function make tiago rotate on himself.
        Angle: total angle of desired rotation
        Theta: angular velocity in rad/s. Positive for counterclockwise, negative for clockwise
        """

        # Define the Twist message for rotation
        rotate_cmd = Twist()
        rotate_cmd.angular.z = theta  # Angular velocity in radians/second (counterclockwise)

        # Calculate rotation duration for 360 degrees
        rotation_duration = angle / abs(rotate_cmd.angular.z) 

        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.Time.now()

        
        while (rospy.Time.now() - start_time).to_sec() < rotation_duration:
            self.cmd_vel_pub.publish(rotate_cmd)
            rate.sleep()
        #self.status_pub.publish(String(data="Rotation terminated"))
        # Stop the robot after rotation
        self.cmd_vel_pub.publish(Twist())  # Publish zero velocity

    def get_targets_callback(self, msg):
        """
        get the target IDs from NodeA from the /target_ids topic
        """
        if not self.target_ids:
            self.target_ids = msg.data  # list of integers (targets ids)
            self.status_pub.publish(String(data=f"Received IDs from Node A: {self.target_ids}"))

    def tag_callback(self, msg):
        """
        Callback to process detected apriltags from the detection node.
        """
        for pose in msg.poses:
            tag_id = int(pose.orientation.w)  # NOTE: we are putting the ID in the .orientation.w field (the orientation of the tags is not required)
            if tag_id in self.target_ids and tag_id not in self.found_tags.keys():  # new target apriltag found
                
                self.found_tags[tag_id] = pose   # store the pose of the tag in map reference frame
                self.status_pub.publish(String(data=f"FOUND TARGET AprilTag: {tag_id}. MISSING TARGETS: {set(self.target_ids)-set(self.found_tags.keys())}")) # feedback to node A

                # check if we finished the task
                if len(self.found_tags) == len(self.target_ids):  # remember that found_tags is a dictionary
                    self.exploration_active = False
                    self.stop_exploration()  # finished the task
                    self.status_pub.publish(String(data="All target tags found! Task is completed."))
                    self.publish_final_results()
                    
  
    def navigation_callback(self, msg):
        """
        Callback to handle feedback from the navigation node.
        """
        feedback = msg.data
        # if current goal is reached we request a new goal to nodeB_exploration
        if feedback == "Goal Reached" and self.exploration_active:
            self.status_pub.publish(String(data=f"GOAL REACHED, requesting a new goal."))
            self.request_new_goal()
        elif feedback == "Goal Failed" and self.exploration_active:         
            self.status_pub.publish(String(data=f"FAILED to reach GOAL, requesting a new goal."))           
            self.request_new_goal()
        # when we exit control law, we just ask for a new goal
        elif feedback == "control law off" and self.exploration_active:         
            self.request_new_goal(rot = False)  # after the control law, no need of random rotation.
        elif feedback == "Time expired" and self.exploration_active:         
            self.status_pub.publish(String(data=f"TIMEOUT reached for current GOAL, requesting a new goal."))           
            self.request_new_goal()

    def request_new_goal(self, rot = True):
        """
        Request the exploration node to generate a new goal
        Before requesting the goal, a random rotation is performed
        """
        #prob_of_rotation = 1/2
        # perform a random rotation with probability 1/3
        #if random.random() < prob_of_rotation:
        if rot:
            self.status_pub.publish(String(data="Performing RANDOM ROTATION."))
            self.rotation(random.uniform(0, 2*math.pi), 1.2)
        exploration_command = String(data="Continue")   
        self.exploration_command_pub.publish(exploration_command)  # send command to nodeB_exploration

    def publish_final_results(self):
        """
        Publish the positions of all requested apriltags, using a message PoseArray.
        The ids are stored in pose.orientation.w
        """
        result_msg = PoseArray()
        result_msg.header.frame_id = "map"
        result_msg.header.stamp = rospy.Time.now()

        for tag_id in self.found_tags.keys():
            pose = self.found_tags[tag_id]
            result_msg.poses.append(pose)  # Append the Pose to the PoseArray

        self.status_pub.publish(String(data=f"Publishing final result to node A"))
        rospy.sleep(2)
        self.final_results_pub.publish(result_msg)  # Send results to Node A

        
    def stop_exploration(self):
        """
        command to the exploration and navigation nodes to stop.
        """
        stop_command = String(data="Stop")
        self.exploration_command_pub.publish(stop_command)

    
    def tilt_camera(self, tilt_angle=-0.5):
        """
        Tilts the camera downward by adjusting the head_2_joint.
        :param tilt_angle: Angle to tilt the camera (in radians, negative for downward tilt).
        """
        # Create a JointTrajectory message
        head_cmd = JointTrajectory()
        head_cmd.joint_names = ['head_1_joint', 'head_2_joint']

        # Create a JointTrajectoryPoint for the desired position
        point = JointTrajectoryPoint()
        point.positions = [0.0, tilt_angle]  # Keep head_1_joint neutral, tilt head_2_joint
        point.time_from_start = rospy.Duration(1.0)  # Move in 1 second

        # Add the point to the trajectory
        head_cmd.points.append(point)

        # Publish the command
        self.status_pub.publish(String(data=f"TILTING CAMERA by {tilt_angle} radians."))
        self.head_pub.publish(head_cmd)
        rospy.sleep(2)  # Allow time for the movement to complete

    def start(self):
        """
        Starts the goal management.
        """
        rospy.loginfo("Starting Goal Management Node...")

        # Wait for target IDs before proceeding
        self.status_pub.publish(String(data=f"Waiting to receive IDs from Node A..."))
        rate = rospy.Rate(10)  # 10 Hz loop
        while not self.target_ids:
            rospy.loginfo_throttle(5, "Still waiting for IDs from Node A...")
            rate.sleep()

        self.exploration_active = True

        # Tilt camera downward at initialization
        self.tilt_camera(tilt_angle=-0.8)  # point camera down for the initial 360 rotation
        rospy.sleep(1)  # Give some time for all nodes to initialize

        self.status_pub.publish(String(data="Performing 360° rotation."))  # update node_A about the new tag found.status_pub
        self.rotation(2*math.pi, 0.8)  # start the task by rotating tiago (the camera) of 360°

        self.tilt_camera(tilt_angle=-0.6)  # point camera on a suitable angle to see apriltags on the floor
        rospy.loginfo("Sending initial 'Continue' command to start exploration.")
        self.exploration_command_pub.publish(String(data="Continue"))

        rospy.spin()
if __name__ == '__main__':
    try:
        node = GoalManagementNode()
        node.start()
    except rospy.ROSInterruptException:
        pass
