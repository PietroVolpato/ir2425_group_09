import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from tf import TransformListener
import math
import actionlib
import numpy as np

#NOT TESTED!!!!!!
class ExplorationNode:
    def __init__ (self):
        rospy.init_node('exploration_node')

        #Subscribe to the topic for the scan
        self.scan_sub = rospy.Subscriber('/scan_raw', LaserScan, self.scan_callback)

        #Client for the navigation stack
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        #Publisher for the goal
        self.goal_pub = rospy.Publisher('/exploration_goal', PoseStamped, queue_size=10)
        self.status_pub = rospy.Publisher('/exploration_status', String, queue_size=10)

        #Transform listener
        self.tf = TransformListener()

        #Variables
        self.min_distance = 1.0
        self.scan_data = None

    def scan_callback(self, msg):
        """"Process the scan data"""
        self.scan_data = msg

    def find_obstacles(self):
        """
        Function to find the obstacles in the scan data
        """
        if self.scan_data is None:
            return []
        
        ranges = np.array(self.scan_data.ranges)
        angle_min = self.scan_data.angle_min
        angle_increment = self.scan_data.angle_increment

        #Find the obstacles
        obstacles = []
        
        for i in range(len(ranges)):
            if abs(ranges[i+1] - ranges[i]) < self.min_distance:
                angle = angle_min + i * angle_increment
                x, y = self.polar_to_cartesian(ranges[i], angle)
                obstacles.append((x, y))

        return obstacles

    def find_free_spaces(self):
        """Analyze scan data to find free spaces to move using the find obstacles function"""
        obstacles = self.find_obstacles()
        free_spaces = []
        for i in range(len(obstacles)-1):
            x1, y1 = obstacles[i]
            x2, y2 = obstacles[i+1]
            x = (x1 + x2) / 2
            y = (y1 + y2) / 2
            free_spaces.append((x, y))
        return free_spaces

    def polar_to_cartesian(self, r, theta):
        x = r * math.cos(theta)
        y = r * math.sin(theta)
        return x, y
    
    def move_to_goal(self, x, y):
        """Move to the given goal"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(goal)
        self.goal_pub.publish(goal.target_pose)
        self.status_pub.publish('Moving to goal')
        self.client.wait_for_result()
        self.status_pub.publish('Goal reached')

    def run(self):
        """Main loop for the exploration node"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            free_spaces = self.find_free_spaces()
            if len(free_spaces) > 0:
                x, y = free_spaces[0]
                self.move_to_goal(x, y)
            rate.sleep()

if __name__ == '__main__':
    node = ExplorationNode()
    node.run()