#!/usr/bin/env python3
import rospy
from tiago_iaslab_simulation.srv import Objs  # Service definition for /apriltag_ids_srv
from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import PoseArray

class NodeA:
    def __init__(self):
        rospy.init_node('nodeA')

        # Publisher to send IDs to Node B
        self.ids_pub = rospy.Publisher('/apriltag_ids', Int32MultiArray, queue_size=10)

        # Subscriber to listen for feedback from Node B
        rospy.Subscriber('/node_b_feedback', String, self.feedback_callback)

        # Subscriber to listen for final cube positions from Node B
        rospy.Subscriber('/final_cube_positions', PoseArray, self.result_callback)

        # Wait for the apriltag_ids_srv service to be available
        rospy.loginfo("Waiting for /apriltag_ids_srv service...")
        rospy.wait_for_service('/apriltag_ids_srv')

        # Call the service to get Apriltag IDs
        self.get_apriltag_ids()

    def get_apriltag_ids(self):
        """
        Calls the /apriltag_ids_srv service to get a list of Apriltag IDs.
        """
        try:
            # Create a service client for /apriltag_ids_srv
            get_ids_service = rospy.ServiceProxy('/apriltag_ids_srv', Objs)

            # Create the service request with 'ready=True'
            rospy.loginfo("Requesting Apriltag IDs...")
            response = get_ids_service(ready=True)

            # Process the response
            target_ids = response.ids
            rospy.loginfo(f"Received Apriltag IDs: {target_ids}")

            # Publish the IDs to Node B
            ids_msg = Int32MultiArray(data=target_ids)
            self.ids_pub.publish(ids_msg)
            rospy.loginfo("Published target IDs to Node B.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call /apriltag_ids_srv service: {e}")

    def feedback_callback(self, msg):
        """
        Callback to process feedback from Node B.
        """
        rospy.loginfo(f"B says: {msg.data}")

    def result_callback(self, msg):
        """
        Callback to process the final results from Node B.
        """
        rospy.loginfo("FINAL CUBE POSITIONS RECEIVED:")
        for pose in msg.poses:
            rospy.loginfo(f"Target AprilTag {pose.orientation.w}: x={pose.position.x}, y={pose.position.y}")

if __name__ == '__main__':
    try:
        node = NodeA()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node A terminated.")
