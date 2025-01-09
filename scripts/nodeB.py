#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseArray, PoseStamped
import tf2_ros 
from tf2_geometry_msgs import do_transform_pose
from std_msgs.msg import String
from ir2425_group_09.msg import Detections  # custom message
from ir2425_group_09.msg import TargetObject  # custom message
import random
import math


class NodeB:
    def __init__(self):
        rospy.init_node('nodeB')

        # Define the publisher to communicate with node C
        self.object_pub = rospy.Publisher('/detected_objects', Detections, queue_size=10)

        # publisher of target pose and id to start picking routine
        self.picking_routine_pub = rospy.Publisher('/picking_routine', TargetObject, queue_size=10)

        # Subscribe to AprilTag detection topic
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)

        # node a tells what are the detections for ("placing" or "picking")
        rospy.Subscriber('/detections_command', String, self.send_detections_callback)

        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.current_detections = None
        self.max_picking_distance = 0.8
    
    def send_detections_callback(self, msg):
        """
        This callback plays when nodeA notify that reached a docking point, thus we are ready to get the detections.
        Is is created a custom message Detections(), that contain the array of transformed (base_link) poses and the array
        of respective object ids. In addition it is passed the id of the target object, which is the object that should be
        grabbed.
        If no detected object is a valid target, then targe
        """

        current_task = msg.data

        detections_msg = Detections()
        detections_msg.header.frame_id = "base_link"  # Target frame
        detections_msg.header.stamp = rospy.Time.now()

        try:
            # Lookup the transformation from the camera frame to the base frame
            transform = self.tf_buffer.lookup_transform("base_link", "xtion_rgb_optical_frame", rospy.Time(0))

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
                        if current_task == "picking":
                            rospy.loginfo(f"Detected reachable obj id {tag_id}, {type}. Planar dist = {planar_distance:.2f}") # print detected id and category of the object
                    
                except Exception as e:
                    rospy.logerr(f"Failed to transform pose for tag ID {tag_id}: {e}")
        except tf2_ros.LookupException as e:
            rospy.loginfo(f"Transform lookup failed: {e}")
        except tf2_ros.ExtrapolationException as e:
            rospy.loginfo(f"Transform extrapolation error: {e}")
        
        # Publish the detections message
        detections_msg.task = current_task
        self.object_pub.publish(detections_msg)  # publish the detections for create planning scene

        if current_task == "picking":

            target_id = random.choice(detections_msg.ids)  # FOR NOW RANDOM CHOICE, will implement the color criterion

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