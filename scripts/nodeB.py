#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseArray, PoseStamped
import tf2_ros 
from tf2_geometry_msgs import do_transform_pose
from std_msgs.msg import String
from ir2425_group_09.msg import Detections  # custom message
from ir2425_group_09.msg import TargetObject  # custom message
from std_msgs.msg import Int32
import secrets
import math
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

class NodeB:
    def __init__(self):
        rospy.init_node('nodeB')

        self.bridge = CvBridge()
        # Simpler HSV ranges with more tolerance
        self.color_ranges = {
            'red': ([0, 242, 165], [5, 255, 245]),  # Similar values to  #B30101
            'green': ([58, 242, 178], [62, 255, 255]),  # Similar values to  #02FF02
            'blue': ([118, 242, 102], [123, 255, 178])  # Similar values to  #0101A2
        }

        self.flag = False
        self.target_color = secrets.choice(['red', 'green', 'blue'])

        # Define the publisher to communicate with node C
        self.object_pub = rospy.Publisher('/detected_objects', Detections, queue_size=10)

        # publisher of target pose and id to start picking routine
        self.picking_routine_pub = rospy.Publisher('/picking_routine', TargetObject, queue_size=10)

        # Subscribe to AprilTag detection topic
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)

        # node a tells what are the detections for ("placing" or "picking")
        rospy.Subscriber('/detections_command', String, self.send_detections_callback)

        rospy.Subscriber('/xtion/rgb/image_raw', Image, self.image_callback)

        self.feedback_pub = rospy.Publisher('/picking_routine_feedback', Int32, queue_size=10)

        self.skip_pub = rospy.Publisher('/skip', String, queue_size=10)

        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.current_detections = None
        self.max_picking_distance = 0.72
        self.current_image = None

    def image_callback(self, msg):
        """
        Callback function for the image subscriber. Converts the ROS image message to an OpenCV image
        """
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Failed to process image: {e}")

    def detect_image_colors(self):
        """
        Detects color in the current image using HSV color ranges.
        Defines three ROIs and for each one:
        - Search for the pixel count of pixels in the three color ranges.
        - Pick the higher pixel count and select the color related to it.
        """
        if self.current_image is None:
            rospy.logwarn("No image available for color detection")
            return []
    
        detected_colors = []
        try:
            hsv_image = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2HSV)
            height, width = hsv_image.shape[:2]
            roi_height = height // 3  
            roi_width = width // 3   

            roi_counter = 1    
    
            for x in range(0, width-1, roi_width):
                x1 = x + 20
                x2 = min(width, x + roi_width - 20)  
                roi = hsv_image[int(roi_height):height - roi_height, x1:x2]

                # Initialize counter for each color
                red_count = 0
                green_count = 0
                blue_count = 0

                # Count the number of pixels in each color range
                # Iterate over each color range
                for color, (lower, upper) in self.color_ranges.items():

                    # Create a mask for the current color
                    mask = cv2.inRange(roi, np.array(lower), np.array(upper))

                    # Count the pixel for the current color
                    pixel_count = np.sum(mask) / 255  # Divide by 255 to get the number of pixels
                    # The mask sets to 255 pixels with matching colors and to 0 non-matching
                    # rospy.loginfo(f"ROI {roi_counter}, Color={color}, Pixels={pixel_count}")
                    
                    if color == 'red':
                        red_count = pixel_count
                    elif color == 'green':
                        green_count = pixel_count
                    elif color == 'blue':
                        blue_count = pixel_count
                    
                # Determine the color with highest pixel count
                max_count = max(red_count, green_count, blue_count)
                if max_count > 45: # Threshold, to increase if there are false positives
                    if max_count == red_count:
                        detected_colors.append('red')
                    elif max_count == green_count:
                        detected_colors.append('green')
                    elif max_count == blue_count:
                        detected_colors.append('blue')

                roi_counter += 1  

    
            return detected_colors
    
        except Exception as e:
            rospy.logerr(f"Error detecting colors: {e}")
            return []
    
    def send_detections_callback(self, msg):
        """
        This callback plays when nodeA notify that reached a docking point, thus we are ready to get the detections.

        It is created a custom message Detections(), that contain the array of transformed (base_link) poses and the array of respective object ids.
        The message in sent to nodeC_planning_scene to create the collision objects.

        If Tiago is about to execute a picking routine, to select the desired object of the wanted color:
        - The object are listed from left to right positions with respect to tiago
        - Through the detect_image_colors function are extracted colors in the same order 
        - Colors and Objects then are matched
        - If a corresponding Object of the wanted color is found proceed with the picking routine
        - Else skip the current docking point
        """

        if not self.flag:
            rospy.loginfo(f"Target color set to: {self.target_color}")
            self.flag = True

        current_task = msg.data

        detections_msg = Detections()
        detections_msg.header.frame_id = "base_link"  # Target frame
        detections_msg.header.stamp = rospy.Time.now()

        try:
            # Lookup the transformation from the camera frame to the base frame
            transform = self.tf_buffer.lookup_transform("base_link", "xtion_rgb_optical_frame", rospy.Time(0))

            object_list = []

            for detection in self.current_detections:
                tag_id = detection.id[0]
                type = self.classify_object(tag_id)

                try:
                    # Create a PoseStamped object from the detected pose
                    pose_stamped = PoseStamped()
                    pose_stamped.header.frame_id = "xtion_rgb_optical_frame"
                    pose_stamped.header.stamp = rospy.Time.now()
                    pose_stamped.pose = detection.pose.pose.pose

                    # Transform the pose to the base frame
                    transformed_pose = do_transform_pose(pose_stamped, transform)

                    x_obj = transformed_pose.pose.position.x
                    y_obj = transformed_pose.pose.position.y
                    planar_distance = math.sqrt(x_obj**2 + y_obj**2)
                    # when picking we want to include only reachable objects
                    if planar_distance < self.max_picking_distance or current_task == "placing":
                        # Add the transformed pose and corresponding ID to the message
                        detections_msg.poses.append(transformed_pose.pose)
                        detections_msg.ids.append(tag_id)
                        detections_msg.types.append(type)

                        # Append to object_list only if the object meets the conditions
                        object_list.append((tag_id, type, transformed_pose.pose, y_obj))
                    
                except Exception as e:
                    rospy.logerr(f"Failed to transform pose for tag ID {tag_id}: {e}")
        except tf2_ros.LookupException as e:
            rospy.loginfo(f"Transform lookup failed: {e}")
        except tf2_ros.ExtrapolationException as e:
            rospy.loginfo(f"Transform extrapolation error: {e}")
        

        # Sort the object_list by y_obj in descending order
        object_list.sort(key=lambda obj: obj[3], reverse=True)  

        if current_task == "picking":
            # Print the sorted contents of object_list
            rospy.loginfo("Object from left to right:")
            for obj in object_list:
                rospy.loginfo(f"Tag ID: {obj[0]}, Object Type: {obj[1]}")

        # Publish the detections message
        detections_msg.task = current_task

        if current_task == "placing":
            self.object_pub.publish(detections_msg)  # publish the detections for create planning scene
        elif current_task == "picking":
            detected_colors = self.detect_image_colors()
            rospy.loginfo(f"Detected colors in order: {detected_colors}")

            valid_targets = []
            for (tag_id, type, pose, _), color in zip(object_list, detected_colors):
                # rospy.loginfo(f"Object ID: {tag_id}, Type: {type}, Color: {color}, Target Color: {self.target_color}")
                if color == self.target_color:
                    valid_targets.append((tag_id))
                    rospy.loginfo(f"Found valid target with ID {tag_id} and correct color {color}")


            if not valid_targets:
                rospy.loginfo("No valid targets with correct color found")
                self.feedback_pub.publish(Int32(data=-1))
                return
            
            if len(valid_targets) == 1:
                self.skip_pub.publish(String(data="skip"))
                

            self.object_pub.publish(detections_msg)  # publish the detections for create planning scene
            target_id = valid_targets[0] #
            index = detections_msg.ids.index(target_id)  # get target index
            target_pose = detections_msg.poses[index] # get target pose to print information
            x = target_pose.position.x
            y = target_pose.position.y
            z = target_pose.position.z
            rospy.loginfo(f"TARGET selected: {target_id}, {self.classify_object(target_id)}. Pose w.r.t. Tiago: ({x:.3f},{y:.3f},{z:.3f})")
            target_msg = TargetObject()  # message containing pose and id of the targer
            target_msg.pose = target_pose
            target_msg.id = target_id
            self.picking_routine_pub.publish(target_msg)  # publish pose and id of target to start picking routine

        return     
    
    def tag_callback(self, msg):
        """
        For efficiency reasons the detection are kept in the node internal state, and will transformed and sent to nodeC_planning_scene only when nodeA 
        communicates that tiago reached a docking point.
        """
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
        
if __name__ == '__main__':
    try:
        node = NodeB()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
