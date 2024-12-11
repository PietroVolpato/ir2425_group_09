#!/usr/bin/env python3
"""
Node B: Apriltag Detection

This node is responsible for:

    - Detecting Apriltags using the camera feed (/tag_detections).
    - Transforming detected tag positions to the map frame.
    - Publishing the transformed positions, with the tag ID encoded in the orientation.w field.

"""

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import String
import tf2_ros
from tf2_geometry_msgs import do_transform_pose


class AprilTagDetectionNode:
    def __init__(self):
        rospy.init_node('apriltag_detection_node', anonymous=True)
        rospy.loginfo("Node B initialized for AprilTag detection.")

        # Publishers
        self.tags_pub = rospy.Publisher('/detected_tags', PoseArray, queue_size=10)
        self.status_pub = rospy.Publisher('/node_b_feedback', String, queue_size=10)

        # Subscribers
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)

        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


        self.found_tags = []

    def tag_callback(self, msg):
        """
        Processes AprilTag detections, transforms their poses to the map frame, and publishes them.
        """

        # Create a PoseArray message for transformed poses
        detected_poses = PoseArray()
        detected_poses.header.frame_id = "map"  # Target frame
        detected_poses.header.stamp = rospy.Time.now()

        try:
            # Lookup the transformation from the camera frame to the map frame
            transform = self.tf_buffer.lookup_transform("map", "xtion_rgb_optical_frame", rospy.Time(0))

            for detection in msg.detections:
                tag_id = detection.id[0]
                if (tag_id not in self.found_tags):                 
                    try:
                        # Create a PoseStamped object from the detected pose
                        pose_stamped = PoseStamped()
                        pose_stamped.header.frame_id = "xtion_rgb_optical_frame"
                        pose_stamped.header.stamp = rospy.Time.now()
                        pose_stamped.pose = detection.pose.pose.pose

                        # Transform the pose to the map frame
                        transformed_pose = do_transform_pose(pose_stamped, transform)

                        # Encode the tag ID in the orientation.w field
                        transformed_pose.pose.orientation.w = float(tag_id)
                        # Add the transformed pose to the PoseArray
                        detected_poses.poses.append(transformed_pose.pose)
                        self.found_tags.append(tag_id)
                        self.status_pub.publish(String(data=f"DETCTED AprilTag: {tag_id}. ALL TAGS found from start: {self.found_tags}"))  # feedback to node A
                    except Exception as e:
                        rospy.logerr(f"Failed to transform pose for tag ID {tag_id}: {e}")

            # Publish the transformed poses if any are detected
            if detected_poses.poses:
                self.tags_pub.publish(detected_poses)               

        except tf2_ros.LookupException as e:
            rospy.loginfo(f"Transform lookup failed:{e}")

        except tf2_ros.ExtrapolationException as e:
            rospy.loginfo(f"Transform extrapolation error: {e}")

    def run(self):
        
        rospy.spin()


if __name__ == '__main__':
    try:
        node = AprilTagDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
