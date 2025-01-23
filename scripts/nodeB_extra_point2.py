#!/usr/bin/env python

import os
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseArray, PoseStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from std_msgs.msg import String
from ir2425_group_09.msg import Detections
from ir2425_group_09.msg import TargetObject
from std_msgs.msg import Int32
import random
import math
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import numpy as np

class NodeB:
    def __init__(self):
        rospy.init_node('nodeB')

        self.bridge = CvBridge()
        # Simpler HSV ranges with more tolerance
        self.color_ranges = {
            'red': ([0, 242, 165], [2, 255, 191]),  # Per valori simili a #B30101
            'green': ([58, 242, 178], [62, 255, 255]),  # Per valori simili a #02FF02
            'blue': ([118, 242, 102], [123, 255, 178])  # Per valori simili a #0101A2
        }

        self.target_color = random.choice(['red', 'green', 'blue'])
        rospy.loginfo(f"Target color set to: {self.target_color}")
        self.debug_image_pub = rospy.Publisher('/debug_image', Image, queue_size=10)

        # Publishers
        self.object_pub = rospy.Publisher('/detected_objects', Detections, queue_size=10)
        self.picking_routine_pub = rospy.Publisher('/picking_routine', TargetObject, queue_size=10)
        self.feedback_pub = rospy.Publisher('/picking_routine_feedback', Int32, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
        rospy.Subscriber('/detections_command', String, self.send_detections_callback)
        rospy.Subscriber('/xtion/rgb/image_raw', Image, self.image_callback)

        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.current_detections = None
        self.current_image = None
        self.max_picking_distance = 0.75


    def image_callback(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Failed to process image: {e}")

    def detect_image_colors(self):
        if self.current_image is None:
            rospy.logwarn("No image available for color detection")
            return []
    
        detected_colors = []
        try:
            hsv_image = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2HSV)
            height, width = hsv_image.shape[:2]
            roi_height = height // 3  # Calculate the midpoint of the image
            roi_width = width // 3     # Divide the image width into 6 parts

            debug_image = self.current_image.copy()  # Copy the image for debugging

    
            for x in range(0, width-1, roi_width):
                x1 = x
                x2 = min(width, x + roi_width)  # Limita l'ultima ROI ai bordi dell'immagine
                roi = hsv_image[roi_height:height, x1:x2]

                for color, (lower, upper) in self.color_ranges.items():
                    mask = cv2.inRange(roi, np.array(lower), np.array(upper))
                    if np.sum(mask) > 3000:
                        detected_colors.append(color)
                        rospy.loginfo(f"Detected {color} in region: {x1}-{x2}")
                        break

                # Draw ROI rectangle on the debug image
                cv2.rectangle(debug_image, (x1, roi_height), (x2, height), (0, 255, 0), 2)

            
            self.save_debug_image(debug_image)
    
            return detected_colors
    
        except Exception as e:
            rospy.logerr(f"Error detecting colors: {e}")
            return []




    def send_detections_callback(self, msg):
        current_task = msg.data
        detections_msg = Detections()
        detections_msg.header.frame_id = "base_link"
        detections_msg.header.stamp = rospy.Time.now()

        try:
            transform = self.tf_buffer.lookup_transform("base_link", "xtion_rgb_optical_frame", rospy.Time(0))

            object_list = []

            for detection in self.current_detections:
                tag_id = detection.id[0]
                object_type = self.classify_object(tag_id)

                try:
                    pose_stamped = PoseStamped()
                    pose_stamped.header.frame_id = "xtion_rgb_optical_frame"
                    pose_stamped.header.stamp = rospy.Time.now()
                    pose_stamped.pose = detection.pose.pose.pose

                    transformed_pose = do_transform_pose(pose_stamped, transform)

                    x_obj = transformed_pose.pose.position.x
                    y_obj = transformed_pose.pose.position.y
                    z_obj = transformed_pose.pose.position.z
                    planar_distance = math.sqrt(x_obj**2 + y_obj**2)

                    if planar_distance < self.max_picking_distance or current_task == "placing":
                        detections_msg.poses.append(transformed_pose.pose)
                        detections_msg.ids.append(tag_id)
                        detections_msg.types.append(object_type)

                        # Append to object_list only if the object meets the conditions
                        object_list.append((tag_id, object_type, transformed_pose.pose, y_obj))


                    if current_task == "picking":
                            rospy.loginfo(f"Detected reachable obj id {tag_id}, {object_type}. Planar dist = {planar_distance:.2f}, x,y = {x_obj},{y_obj}")

                except Exception as e:
                    rospy.logerr(f"Failed to transform pose for tag ID {tag_id}: {e}")

            
            # Sort the object_list by y_obj in descending order
            object_list.sort(key=lambda obj: obj[3], reverse=True)     

            # Print the sorted contents of object_list
            rospy.loginfo("Object from left to right:")
            for obj in object_list:
                rospy.loginfo(f"Tag ID: {obj[0]}, Object Type: {obj[1]}")

       
            detections_msg.task = current_task

            #self.object_pub.publish(detections_msg)

            if current_task == "placing":
                self.object_pub.publish(detections_msg) # publish the detections for create planning scene
            elif current_task == "picking":
                detected_colors = self.detect_image_colors()
                rospy.loginfo(f"Detected colors in order: {detected_colors}")

                valid_targets = []
                for (tag_id, object_type, pose, _), color in zip(object_list, detected_colors):
                    #rospy.loginfo(f"Object ID: {tag_id}, Type: {object_type}, Color: {color}, Target Color: {self.target_color}")
                    if color == self.target_color:
                        valid_targets.append((tag_id, pose))
                        rospy.loginfo(f"Found valid target with ID {tag_id} and correct color {color}")


                if not valid_targets:
                    rospy.loginfo("No valid targets with correct color found")
                    self.feedback_pub.publish(Int32(data=-1))
                    return

                for target_id, target_pose in valid_targets:
                    rospy.loginfo(f"TARGET selected: {target_id}, {self.classify_object(target_id)}. Pose: {target_pose}")
                    target_msg = TargetObject()
                    target_msg.pose = target_pose
                    target_msg.id = target_id
                    self.picking_routine_pub.publish(target_msg)


        except tf2_ros.LookupException as e:
            rospy.loginfo(f"Transform lookup failed: {e}")

    def publish_debug_image(self, image):
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            rospy.logerr(f"Failed to publish debug image: {e}")

    def tag_callback(self, msg):
        self.current_detections = msg.detections

    def classify_object(self, tag_id):
        if tag_id in [1, 2, 3]:
            return "hexagonal prism"
        elif tag_id in [4, 5, 6]:
            return "cube"
        elif tag_id in [7, 8, 9]:
            return "triangular prism"
        elif tag_id == 10:
            return "placing table"

    def save_debug_image(self, image, filename_prefix="debug_image"):
        # Get the path to the Desktop/roi directory
        desktop_path = os.path.expanduser("/home/local/deriste27829/Desktop/roi")
        
        # Ensure the directory exists
        if not os.path.exists(desktop_path):
            os.makedirs(desktop_path)
        
        # Generate the file name
        timestamp = rospy.Time.now().to_sec()
        filename = f"{desktop_path}/{filename_prefix}_{timestamp:.3f}.png"
        
        try:
            # Save the image to the directory
            cv2.imwrite(filename, image)
            rospy.loginfo(f"Saved debug image to {filename}")
        except Exception as e:
            rospy.logerr(f"Failed to save debug image: {e}")


if __name__ == '__main__':
    try:
        node = NodeB()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
