#!/usr/bin/env python3
"""
Node A 

This node is responsible for requesting the IDs of Apriltags via the `/apriltag_ids_srv` service from the `ids_generator_node`.

It then publishes these IDs to Node B.

Throughout the routine, it subscribes to feedback from Node B and displays the current status of the task.

At the end, it listens for the final list of cube positions from Node B and logs them.

Functionality:
1. Request Apriltag IDs from a service and publish them to Node B.
2. Receive feedback on the task's progress from Node B and log it.
3. Receive and log the final cube positions detected by Node B.

"""

import rospy
from tiago_iaslab_simulation.srv import Objs  # Service definition for /apriltag_ids_srv
from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import PoseArray

class NodeA:
    """
    Node A class handles all the functionalities explained above.
    """
    def __init__(self):

        # Node A initialization
        rospy.init_node('node_a')
        rospy.loginfo("Node A initialized.")

        # Publisher for sending Apriltag IDs to Node B
        self.ids_pub = rospy.Publisher('/apriltag_ids', Int32MultiArray, queue_size=10)
        rospy.loginfo("Publisher to /apriltag_ids created.")

        # Subscriber for receiving feedback from Node B
        rospy.Subscriber('/node_b_feedback', String, self.feedback_callback)
        rospy.loginfo("Subscriber to /node_b_feedback created.")

        # Subscriber for receiving final cube positions from Node B
        rospy.Subscriber('/final_cube_positions', PoseArray, self.result_callback)
        rospy.loginfo("Subscriber to /final_cube_positions created.")

        # Wait for the /apriltag_ids_srv service to be available
        rospy.loginfo("Waiting for /apriltag_ids_srv service...")
        rospy.wait_for_service('/apriltag_ids_srv')

        # Call the service to get Apriltag IDs
        self.get_apriltag_ids()

    def get_apriltag_ids(self):
        """
        Calls the `/apriltag_ids_srv` service to retrieve a list of Apriltag IDs from the server.
        Publishes the IDs to Node B for further processing.
        """
        try:
            # Create a client for the /apriltag_ids_srv service
            get_ids_service = rospy.ServiceProxy('/apriltag_ids_srv', Objs)

            # Log the request process
            rospy.loginfo("Requesting Apriltag IDs via service...")

            # Request the IDs with the `ready=True` parameter
            response = get_ids_service(ready=True)

            # Process the response and extract IDs
            target_ids = response.ids
            rospy.loginfo(f"Received Apriltag IDs: {target_ids}")

            # Publish the IDs to Node B
            ids_msg = Int32MultiArray(data=target_ids)
            self.ids_pub.publish(ids_msg)
            rospy.loginfo("Published target IDs to Node B.")
        except rospy.ServiceException as e:
            # Log any issues with the service call
            rospy.logerr(f"Failed to call /apriltag_ids_srv service: {e}")

    def feedback_callback(self, msg):
        """
        Callback to handle feedback messages from Node B.

        Logs the current status of the task as reported by Node B. The feedback 
        may include statuses such as:
        - Robot is moving
        - Robot is scanning for Apriltags
        - Robot has found Apriltags (with specific IDs)
        - Task is completed
        (QUESTI ESEMPI SONO DA SOSTIUTIRE CON QUELLI CHE INSERIREMO NEL NODE B)
        """
        rospy.loginfo(f"Feedback from Node B: {msg.data}")

    def result_callback(self, msg):
        """
        Callback to handle the final list of cube positions from Node B.

        Logs the positions of cubes (as Pose objects) in the map frame. 
        Each cube's position includes x, y, and z coordinates.
        """
        rospy.loginfo("Final cube positions received:")
        for pose in msg.poses:
            rospy.loginfo(f"Cube at: x={pose.position.x}, y={pose.position.y}, z={pose.position.z}")

if __name__ == '__main__':
    try:
        # Instantiate Node A and keep it running
        node = NodeA()
        rospy.spin()
    except rospy.ROSInterruptException:
        # Node A is interrupted or terminated
        rospy.loginfo("Node A terminated.")
