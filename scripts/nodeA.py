#!/usr/bin/env python3
"""
NODE A
This node receives the ids of the five target apriltags from the /apriltag_ids_srv service.
The target ids are published to the goal management node. Constant feedback is provided from the other
nodes, and the information is printed to terminal. All the prints that the user is supposed to see are provided by this node.
Once the task is completed, the positions of the target tags are retrieved by subscribing to the topic final_cube_positions,
and printed to terminal.
"""
import rospy
from tiago_iaslab_simulation.srv import Objs  # Service definition for /apriltag_ids_srv
from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import PoseArray

class NodeA:
    def __init__(self):
        rospy.init_node('nodeA')

        # Publisher to send IDs to Node B
        self.ids_pub = rospy.Publisher('/target_ids', Int32MultiArray, queue_size=10)

        # Subscriber to listen for feedback from Node B
        rospy.Subscriber('/node_b_feedback', String, self.feedback_callback)

        # Subscriber to listen for final cube positions from Node B
        rospy.Subscriber('/final_cube_positions', PoseArray, self.result_callback)

        # Wait for the apriltag_ids_srv service to be available
        rospy.loginfo("Waiting for /apriltag_ids_srv service...")
        rospy.wait_for_service('/apriltag_ids_srv')
        
        self.target_ids = None

        # Call the service to get Apriltag IDs
        self.get_apriltag_ids()

        # Start a timer to periodically publish the target IDs
        rospy.loginfo(f"Publishing TARGET IDs to node B")
        self.timer = rospy.Timer(rospy.Duration(5), self.publish_target_ids)

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
            self.target_ids = response.ids
            rospy.loginfo(f"Received TARGET Apriltag IDs: {self.target_ids}")

        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call /apriltag_ids_srv service: {e}")

    def publish_target_ids(self, event):
        """
        Periodically publish the target IDs to the /apriltag_ids topic.
        """
        msg = Int32MultiArray(data=self.target_ids)
        self.ids_pub.publish(msg)

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
            id = int(pose.orientation.w)
            x = "{:.3f}".format(pose.position.x)
            y = "{:.3f}".format(pose.position.y)
            rospy.loginfo(f"Target AprilTag {id}: x={x}, y={y}")

if __name__ == '__main__':
    try:
        node = NodeA()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node A terminated.")
