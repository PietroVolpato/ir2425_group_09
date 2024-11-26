#!/usr/bin/env python3

"""
Node B Skeleton

This node is responsible for:
1. Navigating through the environment to search for Apriltags on red cubes.
2. Using the camera and laser scanner to detect obstacles and Apriltags.
3. Transforming detected Apriltag poses to the map frame using TF.
4. Sending feedback about the robot's status to Node A.
5. Publishing the final positions of cubes with matching Apriltags to Node A.
"""

import rospy
import tf
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import String, Int32MultiArray
from sensor_msgs.msg import LaserScan, Image
from apriltag_ros.msg import AprilTagDetectionArray

class NodeB:
    """
    Node B handles navigation, Apriltag detection, pose transformation, and communication with Node A.
    """
    def __init__(self):
        # Initialize the node
        rospy.init_node('node_b', anonymous=True)
        rospy.loginfo("Node B initialized.")

        # Subscribe to Apriltag IDs from Node A
        rospy.Subscriber('/apriltag_ids', Int32MultiArray, self.ids_callback)

        # Publisher for feedback to Node A
        self.feedback_pub = rospy.Publisher('/node_b_feedback', String, queue_size=10)

        # Publisher for final cube positions to Node A
        self.result_pub = rospy.Publisher('/final_cube_positions', PoseArray, queue_size=10)

        # Apriltag detections subscriber (from the robot's camera)
        rospy.Subscriber('/camera/apriltag_detections', AprilTagDetectionArray, self.tag_callback)

        # TF listener for pose transformations
        self.tf_listener = tf.TransformListener()

        # Placeholder for Apriltag IDs to be found
        self.target_ids = []

        # Placeholder for final cube positions
        self.cube_positions = PoseArray()

    def ids_callback(self, msg):
        """
        Receives the target Apriltag IDs from Node A.
        TODO:
        - Store the list of target IDs.
        - Log the received IDs for debugging.
        """
		self.target_ids = msg.data
        rospy.loginfo("Received Apriltag IDs: %s", msg.data)
        # Store the target IDs in the class variable
        self.target_ids = msg.data

    def tag_callback(self, msg):
        """
        Processes Apriltag detections.
        TODO:
        - Check if detected IDs match target IDs.
        - For matching IDs, retrieve pose from camera frame.
        - Transform pose to map frame using TF.
        - Store transformed poses in `self.cube_positions`.
        - Publish feedback about detected Apriltags.
        """
        rospy.loginfo("Apriltag detection callback triggered.")
        # Loop through detected Apriltags in the message

    def navigate_and_search(self):
        """
        Handles navigation and scanning tasks.
        TODO:
        - Implement navigation logic (e.g., predefined waypoints or exploration).
        - Use sensors like LaserScan to avoid obstacles.
        - Continuously monitor for Apriltag detections.
        - Publish feedback about navigation progress.
        """
        rospy.loginfo("Starting navigation and search routine.")
        # Example feedback (update as needed during implementation)
        self.send_feedback("Robot is navigating and searching for Apriltags.")

    def transform_pose(self, pose_camera):
        """
        Transforms a pose from the camera frame to the map frame.
        TODO:
        - Use TF to perform the transformation.
        - Handle potential exceptions (e.g., transform not available).
        - Return the transformed pose.
        """
        try:
            # Wait for transform and apply transformation
            rospy.loginfo("Transforming pose from camera frame to map frame.")
            # Add the logic here
            pass
        except tf.Exception as e:
            rospy.logerr(f"TF Error: {e}")
            return None

    def send_feedback(self, message):
        """
        Publishes feedback about the robot's status to Node A.
        TODO:
        - Create a feedback message.
        - Publish the message to `/node_b_feedback`.
        """
        rospy.loginfo(f"Feedback: {message}")
        # Publish the feedback as a String message
        feedback_msg = String(data=message)
        self.feedback_pub.publish(feedback_msg)

    def send_results(self):
        """
        Sends the final list of cube positions to Node A.
        TODO:
        - Publish the `self.cube_positions` PoseArray to `/final_cube_positions`.
        - Send a final feedback message indicating task completion.
        """
        rospy.loginfo("Sending final cube positions to Node A.")
        # Publish the cube positions
        self.result_pub.publish(self.cube_positions)
        self.send_feedback("Detection complete. Final positions sent.")

if __name__ == '__main__':
    try:
        # Instantiate Node B and start the main routine
        node = NodeB()
        rospy.spin()  # Keeps the node alive and listening for messages
    except rospy.ROSInterruptException:
        rospy.loginfo("Node B terminated.")

