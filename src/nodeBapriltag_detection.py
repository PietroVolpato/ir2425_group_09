#!/usr/bin/env python3
"""
Node B: Apriltag Detection

This node is responsible for:

    - Detecting Apriltags using the camera feed (/tag_detections).
    - Publishing the positions of detected tags in the camera frame.

"""

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String


class AprilTagDetectionNode:
    def __init__(self):
        rospy.init_node('apriltag_detection_node', anonymous=True)
        rospy.loginfo("Node B initialized for AprilTag detection.")

        # Publishers
        self.tags_pub = rospy.Publisher('/detected_tags', PoseArray, queue_size=10)
        self.feedback_pub = rospy.Publisher('/node_b_feedback', String, queue_size=10)

        # Subscribers
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)

    def tag_callback(self, msg):
        """
        Processes AprilTag detections and publishes their poses in the camera frame.
        """

        # Create a PoseArray message to store detected poses
        detected_poses = PoseArray()
        detected_poses.header.frame_id = "xtion_rgb_optical_frame"  # Camera frame
        detected_poses.header.stamp = rospy.Time.now()

        # Iterate over detected AprilTags
        for detection in msg.detections:
            tag_id = detection.id[0]
            rospy.loginfo(f"Detected AprilTag ID: {tag_id}")

            # Append the pose of the detected tag to the PoseArray
            detected_poses.poses.append(detection.pose.pose.pose)

        if detected_poses.poses:
            # Publish the detected poses
            self.tags_pub.publish(detected_poses)
            self.feedback_pub.publish(String(data="Published poses of detected tags."))
    

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = AprilTagDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
