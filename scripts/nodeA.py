#!/usr/bin/env python

import rospy
from tiago_iaslab_simulation.srv import Coeffs  # Replace 'your_package' with your ROS package name
from std_msgs.msg import String
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math

class NodeA:
    def __init__(self):
        rospy.init_node("nodeA")

        # Initialize actionlib client
        self.nav_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.nav_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server.")

        # Wait for the service to become available
        rospy.loginfo("Waiting for /straight_line_srv service...")
        rospy.wait_for_service("/straight_line_srv")

        # Create a service proxy
        self.straight_line_srv = rospy.ServiceProxy("/straight_line_srv", Coeffs)
        self.target_points = []  # points where to place the objects (line reference frame)
        self.current_navigation_command = None  # keeps track of current navigation command

        # Define static docking points
        self.docking_points = {
            "corridor exit": (8.7, 0),
            "objects table" : (8.7, -3),
            "picking table" : (8.7, -2)
        }

    def get_coefficients(self):
        try:
            rospy.loginfo("Requesting coefficients from /straight_line_srv...")
            
            # Always set req.ready = True
            response = self.straight_line_srv(ready=True)

            # Retrieve and log coefficients
            m, q = response.coeffs
            rospy.loginfo(f"Received coefficients: m = {m}, q = {q}")
            return m, q

        except rospy.ServiceException as e:
            rospy.logerr("Failed to call service /straight_line_srv: %s", str(e))
            rospy.signal_shutdown("Service call failed")
    
    def send_goal(self, target):

        goal = MoveBaseGoal()
        target = target.lower().strip()
        if target not in ["corridor exit", "objects table", "picking table"]:
            rospy.logerr(f"Unkown target given, goal not defined: {target}")
            return
        
        p = self.docking_points[target] # pick from the dictionary the point corresponding to target
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        if target == "corridor exit":
            goal.target_pose.pose.orientation.z = 0.7  # look table's direction
            goal.target_pose.pose.orientation.w = -0.7
        else:
            goal.target_pose.pose.orientation.w = 0  # look at the table
            goal.target_pose.pose.orientation.z = 1.0  

        #rospy.loginfo(f"Sending goal: {target}")
        self.nav_client.send_goal(goal)

        self.nav_client.wait_for_result()
        result = self.nav_client.get_state()

        if result == actionlib.GoalStatus.SUCCEEDED:   
            if target != "corridor exit":  # print when reaching docking points
                rospy.loginfo(f"Reached DOCKING POINT {target}")
        else:
            rospy.logwarn("Navigation FAILED with status: %s", result)
            self.send_goal(target)

    def set_target_points(self, m, q):
        """
        given the m and q (slope and intercept), computes points on the line equation at a proper distance between each other.
        Use polar coordinates to easly compute points with a specified distance from the line origin.
        """
        distances = [0.2, 0.4, 0.6]
        a = math.atan(m)
        for r in distances:
            x = r * math.cos(a)
            y = r * math.sin(a) + q
            self.target_points.append((x,y))  # save targets in internal state
    
    def run(self):
        m,q = node.get_coefficients()
        self.set_target_points(m,q)
        print(self.target_points)
        self.send_goal("corridor exit")
        self.send_goal("picking table")
        self.send_goal("objects table")
        self.send_goal("picking table")
        self.send_goal("objects table")
        rospy.spin()

if __name__ == "__main__":
    try:
        node = NodeA()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("NodeA terminated.")
