#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
from tiago_iaslab_simulation.srv import Objs
from geometry_msgs.msg import Pose


class GoalManagementNode:
    def __init__(self):
        rospy.init_node('nodeB_goal_management', anonymous=True)

        # Subscriber to get the apriltags which are detected by the camera while Tiago is moving. From nodeB_apriltag_detection
        self.tag_sub = rospy.Subscriber('/detected_tags', PoseArray, self.tag_callback)

        # Subscriber to get the feedback about navigation (e.g. goal reached or aborted). From nodeB_navigation
        self.nav_feedback_sub = rospy.Subscriber('/navigation_feedback', String, self.navigation_callback)

        # Publisher to send commands for the nodeB_exploration ( e.g. "Continue", "Stop").
        self.exploration_command_pub = rospy.Publisher('/exploration_command', String, queue_size=10)

        # Publisher to send to node_A the apriltags positions, when all requested tags are detected
        self.final_results_pub = rospy.Publisher('/final_cube_positions', PoseArray, queue_size=10)

        # Publisher to update node_A (e.g. "task completed")
        self.status_pub = rospy.Publisher('/node_b_feedback', String, queue_size=10)

        # Target AprilTag IDs (from Node A)
        self.target_ids = self.get_target_ids()
        self.found_tags = {}  # disctionary to store positions of found tags
        self.exploration_active = True  # Indicates whether task is ongoing

        rospy.loginfo(f"Goal Management Node initialized with target IDs: {self.target_ids}") # log to show that the IDs were correcly received

    def get_target_ids(self):
        """
        Requests the target IDs from NodeA from the /apriltag_ids_srv service.
        """
        rospy.wait_for_service('/apriltag_ids_srv')
        try:
            apriltag_service = rospy.ServiceProxy('/apriltag_ids_srv', Objs)
            response = apriltag_service(ready=True)
            return response.ids
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call /apriltag_ids_srv: {e}")
            return []

    def tag_callback(self, msg):
        """
        Callback to process detected apriltags from the detection node.
        """
        for pose in msg.poses:
            tag_id = int(pose.orientation.w)  # NOTE: we are putting the ID in the .orientation.w field (the orientation of the tags is not required)
            if tag_id in self.target_ids and tag_id not in self.found_tags:  # new desired apriltag found 
                rospy.loginfo(f"Found target AprilTag ID: {tag_id}")
                self.found_tags[tag_id] = pose.position  # Store the position of the apriltag
                self.status_pub.publish(String(data=f"Tag {tag_id} found!"))  # update node_A about the new tag found 

                # check if we finished the task
                if len(self.found_tags) == len(self.target_ids):  # remember that found_tags is a dictionary
                    rospy.loginfo("All target tags found! Stopping exploration.")
                    self.exploration_active = False
                    self.publish_final_results()
                    self.stop_exploration()  # finished the task


    def navigation_callback(self, msg):
        """
        Callback to handle feedback from the navigation node.
        """
        feedback = msg.data
        rospy.loginfo(f"Navigation feedback received: {feedback}")

        # if current goal is reached we request a new goal to nodeB_exploration
        if feedback == "Goal Reached" and self.exploration_active:
            rospy.loginfo("Goal reached, requesting a new goal.")
            self.request_new_goal()
        elif feedback == "Goal Failed" and self.exploration_active:
            rospy.logwarn("Failed to reach goal, requesting a new goal.")
            self.request_new_goal()

    def request_new_goal(self):
        """
        Request the exploration node to generate a new goal.
        """
        exploration_command = String(data="Continue")   
        self.exploration_command_pub.publish(exploration_command)  # send command to nodeB_exploration

    def publish_final_results(self):
        """
        Publish the positions of all requested apriltags, using a message PoseArray.
        """
        result_msg = PoseArray()
        result_msg.header.frame_id = "map"
        result_msg.header.stamp = rospy.Time.now()
        result_msg.poses = [self.pose_from_position(pos) for pos in self.found_tags.values()]
        self.final_results_pub.publish(result_msg)  # send results to nodeA
        rospy.loginfo("Final results published.")

    def stop_exploration(self):
        """
        command to the exploration and navigation nodes to stop.
        """
        stop_command = String(data="Stop")
        self.exploration_command_pub.publish(stop_command)
        self.status_pub.publish(String(data="Exploration completed."))

    def pose_from_position(self, position):
        """
        Helper to create a Pose message from a position, since data are returned through a PoseArray.
        """
        pose = Pose()
        pose.position = position
        pose.orientation.w = 1.0  # neutral orientation (orientation is not defined for this task)
        return pose

if __name__ == '__main__':
    try:
        node = GoalManagementNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
