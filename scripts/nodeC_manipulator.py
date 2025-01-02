import rospy
from geometry_msgs.msg import PoseStamped
from moveit_commander import PlanningSceneInterface, RobotCommander, MoveGroupCommander
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from ir2425_group_09.msg import Detections
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import pi

class NodeC_manipulator:
    def __init__ (self):
        rospy.init_node('nodeC_manipulator')

        # Subscribe to the nav_goal topic
        rospy.Subscriber('/start_manipulation', CollisionObject, self.manipulation_callback)

        # Initialize actionlib client
        self.nav_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.nav_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server.")

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()

        self.arm_group = MoveGroupCommander("arm")
        self.arm_torso_group = MoveGroupCommander("arm_torso")
        self.gripper_group = MoveGroupCommander("gripper")

        self.object_list = {'1' : 'hexagonal prism', '2': 'hexagonal prism', '3' : 'hexagonal prism',
                            '4': 'cube', '5': 'cube', '6': 'cube',
                            '7': 'traingular prism', '8': 'traingular prism', '9': 'traingular prism'}
        
        rospy.sleep(10.0)
        self.initial_config()

    def initial_config(self):
        """
        Move the arm to initial configuration with proper state handling
        """
        rospy.loginfo("Moving arm to initial configuration")
        
        try:
            # Get current state and joint values
            self.arm_torso_group.set_start_state_to_current_state()
            current_joint_values = self.arm_group.get_current_joint_values()
            
            # Define target configuration
            configuration = {
                'torso_lift_joint': 0.35,
                'arm_1_joint': pi / 2,
                'arm_2_joint': 1.0,
                'arm_3_joint': 0.0,
                'arm_4_joint': 0.0,
                'arm_5_joint': current_joint_values[4],
                'arm_6_joint': current_joint_values[5],
                'arm_7_joint': current_joint_values[6]
            }
            
            # Set planning parameters
            self.arm_torso_group.set_goal_joint_tolerance(0.02)
            self.arm_torso_group.set_planning_time(5.0)
            
            # Set target and plan
            self.arm_torso_group.set_joint_value_target(configuration)
            plan = self.arm_torso_group.plan()
            
            # Handle different return types based on MoveIt version
            if isinstance(plan, tuple):
                success = plan[0]
                trajectory = plan[1]
            else:
                success = True
                trajectory = plan
                
            if success and trajectory:
                rospy.loginfo("Planning successful, executing trajectory...")
                self.arm_torso_group.execute(trajectory, wait=True)
                rospy.loginfo("Initial configuration reached")
                return True
            else:
                rospy.logerr("Planning failed")
                return False
                
        except Exception as e:
            rospy.logerr(f"Error moving to initial configuration: {str(e)}")
            return False

    def manipulation_callback(self, msg):
        """
        Start the manipulation process
        """
        rospy.loginfo("Received manipulation command")
        target = msg
        
        # Identify the object
        type = self.identify_object(target)
        rospy.loginfo(f"Identified target object: {type}")
        
        # If it is a cube or a triangular prim, moive the arm 10 cm above the object
        # if type == 'cube' or type == 'triangular prism':
        # self.initial_config()
        self.move_arm_above_object(target)


    def move_arm_above_object(self, target):
        """
        Move the arm 10 cm above the object
        """
        rospy.loginfo("Moving arm above object")
        try:
            # Get the pose of the object
            pose = target.pose
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.pose = pose

            # Move the arm 10 cm above the object
            pose_stamped.pose.position.z += 0.1

            self.arm_torso_group.set_pose_target(pose_stamped)
            
            # Get plan (in newer MoveIt versions, plan() returns only the plan)
            plan = self.arm_group.plan()
            
            if isinstance(plan, tuple):
                # MoveIt 1.1+ returns tuple (success, trajectory)
                success = plan[0]
                trajectory = plan[1]
            else:
                # Older MoveIt versions return only trajectory
                success = True
                trajectory = plan
                
            if success:
                rospy.loginfo("Planning successful, executing trajectory...")
                self.arm_group.execute(trajectory, wait=True)
                rospy.loginfo("Arm moved above object")
            else:
                rospy.logerr("Planning failed!")
                
        except Exception as e:
            rospy.logerr(f"Error moving arm: {str(e)}")


    def identify_object(self, target):
        """
        Identify the object to be manipulated
        """
        return self.object_list[target.id]

    def navigate_to_placing_table(self):
        """
        Navigate to the placing table
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 8.5
        goal.target_pose.pose.position.y = -2.0

        self.nav_client.send_goal(goal)
        self.nav_client.wait_for_result()

        result = self.nav_client.get_result()

        if result == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Navigation to placing table succeeded.")

if __name__ == '__main__':
    try:
        node = NodeC_manipulator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass