#!/usr/bin/env python3
"""
Node B: Apriltag Detection

This node is responsible for 

	- detecting Apriltag using the camera feed (/xtion/...)
	- matching detected tags with the target IDs provided by Node A
	- publish the position of matching tags

"""

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class AprilTagDetectionNode:
    def __init__(self):
        rospy.init_node('apriltag_detection_node', anonymous=True)
		rospy.loginfo("Node B initialized for AprilTag detection.")

		"""
		Publishers
		"""
        # Publisher to publish detected tag poses to the '/final_cube_positions' topic
        self.tags_pub = rospy.Publisher('/final_cube_positions', PoseArray, queue_size=10)
        # Publisher to send feedback messages to the '/node_b_feedback' topic
        self.feedback_pub = rospy.Publisher('/node_b_feedback', String, queue_size=10)

		"""
		Subscribers
		"""
        # Subscribes to the '/tag_detections' topic to receive messages containing detected tags
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
		# Subscribes to the '/apriltag_ids ' topic to receive the target Apriltag IDs from Node A
		rospy.Subscriber('/apriltag_ids', Int32MultiArray, self.update_target_ids)

		# TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Target IDs to look for during detection (provided by Node A)
        self.target_ids = []
		rospy.loginfo("Waiting for Apriltag IDs from Node A...")

	def update_target_ids(self, msg):
		"""
        Updates the list of target Apriltag IDs.
        """
		self.target_ids = msg.data
		rospy.loginfo(f"Updated target IDs: {self.target_ids}")
		self.feedback_pub.publish(String(data="Received target IDs from Node A."))

    def tag_callback(self, msg):
        """
        Processes AprilTag detections, matches target IDs, and publishes transformed poses.
        """
		self.feedback_pub.publish(String(data="Scanning for AprilTags."))

        # Creates a PoseArray message to store the poses of detected tags
        detected_poses = PoseArray()

        # Sets the frame of reference for the detected poses to "map"
        detected_poses.header.frame_id = "map"

        # Sets the timestamp for the detected poses to the current ROS time
        detected_poses.header.stamp = rospy.Time.now()

		try:
            # Lookup transformation from camera frame to map frame
            transform = self.tf_buffer.lookup_transform("map", "xtion_rgb_optical_frame", rospy.Time(0))

		    # Iterates over each detected AprilTag in the received message
		    for detection in msg.detections:
		        tag_id = detection.id[0]

		        # Checks if the detected tag ID is in the list of target IDs
		        if tag_id in self.target_ids:
		            rospy.loginfo(f"Detected target AprilTag ID: {tag_id}")

					# Transform pose to "map" frame
                    pose_transformed = do_transform_pose(detection.pose.pose.pose, transform)
					# Adds the pose of the detected target tag to the PoseArray
                    detected_poses.poses.append(pose_transformed.pose)

		    # Checks if any target tags were detected
		    if detected_poses.poses:
		        self.tags_pub.publish(detected_poses)
                self.feedback_pub.publish(String(data=f"Published poses of detected tags: {len(detected_poses.poses)}"))

            else:
                self.feedback_pub.publish(String(data="No target AprilTags detected."))

		except tf2_ros.LookupException as e:
            rospy.logerr(f"Transform error: {e}")
            self.feedback_pub.publish(String(data="Error in transforming tag poses."))


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = AprilTagDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
