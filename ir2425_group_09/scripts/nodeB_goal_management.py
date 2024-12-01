#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
from tiago_iaslab_simulation.srv import Objs
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist


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

        # Publisher to update node_A (e.g. "task completed")
        self.status_pub = rospy.Publisher('/node_b_feedback', String, queue_size=10)

        # publisher to send direct velocity commands
        self.cmd_vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)


        self.head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10) # to move the camera angle
        rospy.sleep(1)  # Wait for the publisher to initialize
        # TF listener
        #self.tf_listener = tf.TransformListener()

        # Target AprilTag IDs (from Node A)
        self.target_ids = self.get_target_ids()
        self.found_tags = {}  # disctionary to store positions of found tags
        self.exploration_active = True  # Indicates whether task is ongoing

        rospy.loginfo(f"Goal Management Node initialized with target IDs: {self.target_ids}") # log to show that the IDs were correcly received


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

        rospy.loginfo("Starting 360-degree rotation...")
        while (rospy.Time.now() - start_time).to_sec() < rotation_duration:
            self.cmd_vel_pub.publish(rotate_cmd)
            rate.sleep()

        # Stop the robot after rotation
        rospy.loginfo("Stopping rotation...")
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
            return response.ids
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call /apriltag_ids_srv: {e}")
            return []

    def tag_callback(self, msg):
        """
        Callback to process detected apriltags from the detection node.
        """
        for pose in msg.poses:
            tag_id = int(pose.orientation.w)  # NOTE: we are putting the ID in the .orientation.w field (the orientation of the tags is not required)
            if tag_id in self.target_ids and tag_id not in self.found_tags:  # new desired apriltag found
                #pose_in_map_frame = self.transform_to_map_frame(pose)    # Transform the pose to the map frame
                rospy.loginfo(f"Found target AprilTag ID: {tag_id}")
                self.found_tags[tag_id] = pose.position   # store the pose of the tag in map reference frame
                self.status_pub.publish(String(data=f"Tag {tag_id} found!"))  # update node_A about the new tag found 

                # check if we finished the task
                if len(self.found_tags) == len(self.target_ids):  # remember that found_tags is a dictionary
                    rospy.loginfo("All target tags found! Stopping exploration.")
                    self.exploration_active = False
                    self.publish_final_results()
                    self.stop_exploration()  # finished the task

    #def transform_to_map_frame(self, pose):
        """
        Transform a pose from the camera frame to the map frame using tf.
        """
     #   try:
            # Prepare PoseStamped from the given pose
      #      pose_stamped = PoseStamped()
       #     pose_stamped.header.frame_id = "xtion_rgb_optical_frame"  # Camera frame
        #    pose_stamped.header.stamp = rospy.Time.now()
         #   pose_stamped.pose = pose

            # Wait for the transform to become available
          #  self.tf_listener.waitForTransform("map", "xtion_rgb_optical_frame", rospy.Time(0), rospy.Duration(2.0))
           # pose_in_map_frame = self.tf_listener.transformPose("map", pose_stamped)
            #return pose_in_map_frame
        #except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
         #   rospy.logwarn(f"Transform failed: {e}")
        #return None
    
    def navigation_callback(self, msg):
        """
        Callback to handle feedback from the navigation node.
        """
        feedback = msg.data
        rospy.loginfo(f"Navigation feedback received: {feedback}")

        # if current goal is reached we request a new goal to nodeB_exploration
        if feedback == "Goal Reached" and self.exploration_active:
            rospy.loginfo("Goal reached, requesting a new goal.")
            self.request_new_goal()
        elif feedback == "Goal Failed" and self.exploration_active:
            rospy.logwarn("Failed to reach goal, requesting a new goal.")
            self.request_new_goal()

    def request_new_goal(self):
        """
        Request the exploration node to generate a new goal.
        """
        exploration_command = String(data="Continue")   
        self.exploration_command_pub.publish(exploration_command)  # send command to nodeB_exploration

    def publish_final_results(self):
        """
        Publish the positions of all requested apriltags, using a message PoseArray.
        """
        result_msg = PoseArray()
        result_msg.header.frame_id = "map"
        result_msg.header.stamp = rospy.Time.now()
        result_msg.poses = [self.pose_from_position(pos) for pos in self.found_tags.values()]
        self.final_results_pub.publish(result_msg)  # send results to nodeA
        rospy.loginfo("Final results published.")

    def stop_exploration(self):
        """
        command to the exploration and navigation nodes to stop.
        """
        stop_command = String(data="Stop")
        self.exploration_command_pub.publish(stop_command)
        self.status_pub.publish(String(data="Exploration completed."))

    def pose_from_position(self, position):
        """
        Helper to create a Pose message from a position, since data are returned through a PoseArray.
        """
        pose = Pose()
        pose.position = position
        pose.orientation.w = 1.0  # neutral orientation (orientation is not defined for this task)
        return pose
    
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
        rospy.loginfo(f"Tilting camera downward to angle {tilt_angle} radians.")
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

        self.rotate_360()  # start the task by rotating tiago (the camera) of 360°

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
