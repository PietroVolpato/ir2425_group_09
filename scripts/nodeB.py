#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray  # Assuming apriltag_ros package
from geometry_msgs.msg import PoseArray, PoseStamped
import tf2_ros 
from tf2_geometry_msgs import do_transform_pose
from std_msgs.msg import String

class NodeB:
    def __init__(self):
        rospy.init_node('node_b')

        # Define the publisher to communicate with node C
        self.object_pub = rospy.Publisher('/detected_objects', String, queue_size=10)

        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribe to AprilTag detection topic
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)

    def tag_callback(self, msg):
        detected_poses = PoseArray()
        detected_poses.header.frame_id = "map"  # Target frame
        detected_poses.header.stamp = rospy.Time.now()

        try:
            # Lookup the transformation from the camera frame to the map frame
            transform = self.tf_buffer.lookup_transform("map", "xtion_rgb_optical_frame", rospy.Time(0))

            for detection in msg.detections:
                tag_id = detection.id[0]

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
                    self.status_pub.publish(String(data=f"DETCTED AprilTag: {tag_id}."))  # feedback to node A
                
                    # Classify the object
                    object_shape, dimensions = self.classify_object(tag_id)

                    # Publish the object information
                    object_info = self.object_info(tag_id, pose_robot, object_shape, dimensions)
                    self.object_pub.publish(object_info)

                    # Print the detected object information to the terminal
                    rospy.loginfo(f"Detected Object: {object_info}")

                except Exception as e:
                    rospy.logerr(f"Failed to transform pose for tag ID {tag_id}: {e}")
        except tf2_ros.LookupException as e:
            rospy.loginfo(f"Transform lookup failed:{e}")

        except tf2_ros.ExtrapolationException as e:
            rospy.loginfo(f"Transform extrapolation error: {e}")

    def classify_object(self, tag_id):
        if tag_id in [1, 2, 3]:
            return "hexagonal", {"h": 0.2, "r": 0.05}
        elif tag_id in [4, 5, 6]:
            return "cube", {"l": 0.05}
        elif tag_id in [7, 8, 9]:
            return "triangular_prism", {"b": 0.05, "h": 0.035, "L": 0.07}
        else:
            return "unknown", {}

    def object_info(self, tag_id, pose, shape, dimensions):
        position = pose.pose.position
        orientation = pose.pose.orientation

        return (f"Tag ID: {tag_id}, Shape: {shape}, Dimensions: {dimensions}, "
                f"Position: ({position.x}, {position.y}, {position.z}), "
                f"Orientation: ({orientation.x}, {orientation.y}, {orientation.z}, {orientation.w})")

if __name__ == '__main__':
    try:
        node = NodeB()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
