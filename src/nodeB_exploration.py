import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math
import numpy as np

#NOT TESTED!!!!!!
class ExplorationNode:
    def __init__ (self):
        rospy.init_node('exploration_node')

        #Publisher
        self.goal_pub = rospy.Publisher('/exploration_goal', PoseStamped, queue_size=10)
        self.exploration_pub = rospy.Publisher('/exploration_status', String, queue_size=10)

        #Subscriber
        rospy.Subscriber('/scan_raw', LaserScan, self.laser_callback)

        #Variables
        self.safe_distance = 0.5
        self.visited_goal = []

    def laser_callback(self, msg):
        """Process the laser scan data to find the nearest obstacle"""
        #Find the nearest obstacle
        min_dist = min(msg.ranges)
        min_index = np.argmin(msg.ranges)
        angle = msg.angle_min + min_index * msg.angle_increment

        #Check if the nearest obstacle is close enough
        if min_dist < self.safe_distance:
            #Find the angle of the obstacle
            angle = msg.angle_min + min_index * msg.angle_increment
            #Find the position of the obstacle
            x = min_dist * math.cos(angle)
            y = min_dist * math.sin(angle)
            #Find the goal position
            goal_x = x + 0.5
            goal_y = y + 0.5
            #Publish the goal
            self.publish_goal(goal_x, goal_y)

    def publish_goal(self, x, y):
        """Publish the goal"""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        self.goal_pub.publish(goal)

    def run(self):
        """Run the exploration node"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            #Check if the goal is reached
            if self.is_goal_reached():
                #Publish the goal
                self.publish_goal()
            rate.sleep()

        rospy.spin()

    def is_goal_reached(self):
        """Check if the goal is reached"""
        pass

if __name__ == '__main__':
    node = ExplorationNode()
    node.run()