#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray  # Assuming apriltag_ros package
from geometry_msgs.msg import PoseArray, PoseStamped
import tf2_ros 
from tf2_geometry_msgs import do_transform_pose
from std_msgs.msg import String
from ir2425_group_09.msg import Detections  # custom message
import random


class NodeB:
    def __init__(self):
        rospy.init_node('nodeB')

        # Define the publisher to communicate with node C
        self.object_pub = rospy.Publisher('/detected_objects', Detections, queue_size=10)

        # Subscribe to AprilTag detection topic
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)

        rospy.Subscriber('/ready_detection', String, self.send_detections_callback)

        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.current_detections = None
    
    def send_detections_callback(self, msg):
        """
        This callback plays when nodeA notify that reached a docking point, thus we are ready to get the detections.
        Is is created a custom message Detections(), that contain the array of transformed (base_link) poses and the array
        of respective object ids. In addition it is passed the id of the target object, which is the object that should be
        grabbed.
        If no detected object is a valid target, then targe
        
        """
        detections_msg = Detections()
        detections_msg.header.frame_id = "base_link"  # Target frame
        detections_msg.header.stamp = rospy.Time.now()

        try:
            # Lookup the transformation from the camera frame to the base frame
            transform = self.tf_buffer.lookup_transform("base_link", "xtion_rgb_optical_frame", rospy.Time(0))
            rospy.loginfo(f"number of DETECTIONS: {len(self.current_detections)}")

            for detection in self.current_detections:
                tag_id = detection.id[0]

                try:
                    # Create a PoseStamped object from the detected pose
                    pose_stamped = PoseStamped()
                    pose_stamped.header.frame_id = "xtion_rgb_optical_frame"
                    pose_stamped.header.stamp = rospy.Time.now()
                    pose_stamped.pose = detection.pose.pose.pose

                    # Transform the pose to the base frame
                    transformed_pose = do_transform_pose(pose_stamped, transform)

                    # Add the transformed pose and corresponding ID to the message
                    detections_msg.poses.append(transformed_pose.pose)
                    detections_msg.ids.append(tag_id)

                    rospy.loginfo(self.classify_object(tag_id)) # print detected id and category of the object
                    
                except Exception as e:
                    rospy.logerr(f"Failed to transform pose for tag ID {tag_id}: {e}")

            detections_msg.target = random.choice(detections_msg.ids)  # FOR NOW RANDOM CHOICE, will implement the color criterion

            # Publish the combined message
            self.object_pub.publish(detections_msg)

        except tf2_ros.LookupException as e:
            rospy.loginfo(f"Transform lookup failed: {e}")
        except tf2_ros.ExtrapolationException as e:
            rospy.loginfo(f"Transform extrapolation error: {e}")

    def tag_callback(self, msg):
        """
        For efficiency reasons the detection are kept in the node internal state, and will transformed and sent to node C only when nodeA 
        communicates that tiago reached a docking point.
        """
        self.current_detections = msg.detections

    def classify_object(self, tag_id):
        if tag_id in [1, 2, 3]:
            return f"{tag_id}, type = hexagonal prism"
        elif tag_id in [4, 5, 6]:
            return f"{tag_id}, type = cube"
        elif tag_id in [7, 8, 9]:
            return f"{tag_id}, type = triangular prism"
        else:
            return f"{tag_id}, type = table"

if __name__ == '__main__':
    try:
        node = NodeB()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass