#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
import tf
from tiago_iaslab_simulation.srv import Objs  # Import the service definition to interact with Node A
from std_msgs.msg import Int32MultiArray

class NavigationNode:
    def __init__(self):
        rospy.init_node('navigation_node')

        # Publishers for robot status and cube positions
        self.status_pub = rospy.Publisher('/robot_status', String, queue_size=10)
        self.cube_positions_pub = rospy.Publisher('/final_cube_positions', PoseArray, queue_size=10)

        # TF listener for pose transformation
        self.tf_listener = tf.TransformListener()

        # Subscriber to receive Apriltag IDs from Node A
        rospy.Subscriber('/apriltag_ids', Int32MultiArray, self.apriltag_ids_callback)

        # Subscriber to listen to camera data for AprilTag detection
        rospy.Subscriber('/camera_topic', Image, self.camera_callback)

        # Store the target Apriltag IDs received from Node A
        self.target_ids = []

        rospy.loginfo("Node B initialized and waiting for Apriltag IDs from Node A.")

    def apriltag_ids_callback(self, msg):
        """
        Callback to receive Apriltag IDs from Node A.
        """
        self.target_ids = msg.data  # Store the received IDs
        rospy.loginfo(f"Received Apriltag IDs: {self.target_ids}")

    def navigate_environment(self):
        self.publish_status("Robot is navigating...")
        # Implement navigation logic (e.g., move_base or custom logic)
        rospy.sleep(1)  # Placeholder for actual navigation logic

    def camera_callback(self, data):
        """
        Callback to process camera data and detect AprilTags.
        """
        detected_tags = []  # Replace with actual AprilTag detection logic
        for tag in detected_tags:
            if tag.id in self.target_ids:
                self.process_tag(tag)

    def process_tag(self, tag):
        """
        Process each detected AprilTag, transform pose and publish final position.
        """
        self.publish_status(f"Detected AprilTag ID: {tag.id}")

        try:
            # Transform tag pose from camera frame to map frame
            tag_pose_map = self.tf_listener.transformPose("map", tag.pose)

            # Publish the cube's final position
            self.publish_cube_position(tag.id, tag_pose_map)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("TF Transform failed")

    def publish_status(self, status):
        """
        Publish robot's status to /robot_status topic.
        """
        self.status_pub.publish(String(data=status))

    def publish_cube_position(self, tag_id, pose):
        """
        Publish final position of the cube corresponding to the detected AprilTag.
        """
        # Assuming `pose` is of type `PoseStamped`
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"  # Map frame
        pose_msg.pose = pose.pose  # Pose transformation from camera to map frame

        # Publish the PoseArray with final positions
        pose_array = PoseArray()
        pose_array.header = pose_msg.header
        pose_array.poses.append(pose_msg.pose)
        self.cube_positions_pub.publish(pose_array)

if __name__ == '__main__':
    try:
        node = NavigationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation Node B terminated.")

