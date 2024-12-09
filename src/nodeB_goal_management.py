#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
from tiago_iaslab_simulation.srv import Objs
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
import math
import random


class GoalManagementNode:
    def __init__(self):
        rospy.init_node('nodeB_goal_management', anonymous=True)

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

        rospy.sleep(1)  # Wait for the publisher to initialize

        self.target_ids = self.get_target_ids()
        self.found_tags = {}  # disctionary to store positions of found tags
        self.exploration_active = True  # Indicates whether task is ongoing

        self.status_pub.publish(String(data=f"Targed IDs received: {self.target_ids}"))


    def rotation(self, angle):
        """
        This function rotates tiago counterclockwise, by the angle specified as parameter.
        An appropriate angular velocity is applied to make tiago rotate.
        """

        # Define the Twist message for rotation
        rotate_cmd = Twist()
        rotate_cmd.angular.z = 0.8  # Angular velocity in radians/second (counterclockwise)

        # Calculate rotation duration for 360 degrees
        rotation_duration = angle / abs(rotate_cmd.angular.z) 

        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.Time.now()

        
        while (rospy.Time.now() - start_time).to_sec() < rotation_duration:
            self.cmd_vel_pub.publish(rotate_cmd)
            rate.sleep()
        self.status_pub.publish(String(data="Rotation terminated"))
        # Stop the robot after rotation
        self.cmd_vel_pub.publish(Twist())  # Publish zero velocity
        rospy.sleep(1)

    def get_target_ids(self):
        """
        Requests the target IDs from NodeA from the /apriltag_ids_srv service.
        """
        rospy.wait_for_service('/apriltag_ids_srv')
        try:
            apriltag_service = rospy.ServiceProxy('/apriltag_ids_srv', Objs)
            response = apriltag_service(ready=True)
            return [int(id) for id in response.ids]   # IMPORTANT CASTING TO INT.
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call /apriltag_ids_srv: {e}")
            return []

    def tag_callback(self, msg):
        """
        Callback to process detected apriltags from the detection node.
        """
        for pose in msg.poses:
            tag_id = int(pose.orientation.w)  # NOTE: we are putting the ID in the .orientation.w field (the orientation of the tags is not required)
            if tag_id in self.target_ids and tag_id not in self.found_tags.keys():  # new target apriltag found
                
                self.status_pub.publish(String(data=f"Found TARGET AprilTag ID: {tag_id}")) # feedback to node A

                self.found_tags[tag_id] = pose   # store the pose of the tag in map reference frame

                self.status_pub.publish(String(data=f"Missing TARGET AprilTags: {set(self.target_ids)-set(self.found_tags.keys())}")) # feedback to node A

                # check if we finished the task
                if len(self.found_tags) == len(self.target_ids):  # remember that found_tags is a dictionary
                    self.status_pub.publish(String(data="All target tags found! Task is completed."))
                    self.exploration_active = False
                    self.publish_final_results()
                    self.stop_exploration()  # finished the task

   
    def navigation_callback(self, msg):
        """
        Callback to handle feedback from the navigation node.
        """
        feedback = msg.data
        # if current goal is reached we request a new goal to nodeB_exploration
        if feedback == "Goal Reached" and self.exploration_active:
            self.status_pub.publish(String(data=f"Goal reached, requesting a new goal."))
            self.request_new_goal()
        elif feedback == "Goal Failed" and self.exploration_active:         
            self.status_pub.publish(String(data=f"Failed to reach goal, requesting a new goal."))           
            self.request_new_goal()
        elif feedback == "Time expired" and self.exploration_active:         
            self.status_pub.publish(String(data=f"Timout reached for current goal, requesting a new goal."))           
            self.request_new_goal()

    def request_new_goal(self):
        """
        Request the exploration node to generate a new goal
        Before requesting the goal, a random rotation is performed
        """
        self.status_pub.publish(String(data="Performing random rotation."))  
        self.rotation(random.uniform(0, 2 * math.pi))
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
        for id in self.found_tags.keys():
            result_msg.append(self.found_tags[id])
        self.status_pub.publish(String(data=f"Publishing final result to node A"))
        rospy.sleep(2)
        self.final_results_pub.publish(result_msg)  # send results to nodeA
        
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
        self.status_pub.publish(String(data=f"Tilting camera downward to angle {tilt_angle} radians."))
        self.head_pub.publish(head_cmd)
        rospy.sleep(2)  # Allow time for the movement to complete

    def start(self):
        """
        Starts the goal management routine.
        """
        rospy.loginfo("Starting Goal Management Node...")

        # Tilt camera downward at initialization
        self.tilt_camera(tilt_angle=-0.8)  # point camera down for the initial 360 rotation
        rospy.sleep(1)  # Give some time for all nodes to initialize

        self.status_pub.publish(String(data="Performing 360° rotation."))  # update node_A about the new tag found.status_pub
        self.rotation(2*math.pi)  # start the task by rotating tiago (the camera) of 360°

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
