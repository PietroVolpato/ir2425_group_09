import rospy
from geometry_msgs.msg import PoseStamped, Pose
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
from tf.transformations import quaternion_from_euler

class NodeC_manipulator:
    def __init__ (self):
        rospy.init_node('nodeC_manipulator')

        # Subscribe to the nav_goal topic
        rospy.Subscriber('/start_manipulation', PoseStamped, self.manipulation_callback)

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

        self.arm_torso_group.set_max_velocity_scaling_factor(0.5)
        self.arm_torso_group.set_max_acceleration_scaling_factor(0.5)
        self.arm_torso_group.set_planning_time(10.0)  
        self.arm_torso_group.set_num_planning_attempts(30)  
        self.arm_torso_group.set_goal_position_tolerance(0.01)
        self.arm_torso_group.set_goal_orientation_tolerance(0.01)

        self.object_list = {'1' : 'hexagonal prism', '2': 'hexagonal prism', '3' : 'hexagonal prism',
                            '4': 'cube', '5': 'cube', '6': 'cube',
                            '7': 'traingular prism', '8': 'traingular prism', '9': 'traingular prism'}
        
        rospy.sleep(15.0)

        self.initial_config()

    def initial_config(self):
        """
        Move the arm to initial configuration with proper state handling
        """
        rospy.loginfo("Moving arm to initial configuration")
        
        # Get current state and joint values
        self.arm_torso_group.set_start_state_to_current_state()
        current_joint_values = self.arm_group.get_current_joint_values()
            
        # Define target configuration
        configuration = {
                'torso_lift_joint': 0.35,
                'arm_1_joint': pi / 2,
                'arm_2_joint': 0.5,
                'arm_3_joint': 0.0,
                'arm_4_joint': 0.0,
                'arm_5_joint': current_joint_values[4],
                'arm_6_joint': current_joint_values[5],
                'arm_7_joint': current_joint_values[6]}
            
        # Set target and plan
        self.arm_torso_group.set_joint_value_target(configuration)

        self.arm_torso_group.go(wait=True)
        self.arm_torso_group.stop()

    def manipulation_callback(self, msg):
        """
        Start the manipulation process
        """
        rospy.loginfo("Received manipulation command")
        target_pose = msg
        
        # Identify the object
        # type = self.identify_object(target)
        # rospy.loginfo(f"Identified target object: {type}")
        self.move_arm_above_object(target_pose)


    def move_arm_above_object(self, target_pose):
        """
        Move the arm 10 cm above the object
        """
        rospy.loginfo("Moving arm above object")
        print(f"Target pose: {target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z}")
        try:
            target_pose.pose.position.z += 0.3
            q = quaternion_from_euler(-pi, 0, pi/2)
            target_pose.pose.orientation.x = q[0]
            target_pose.pose.orientation.y = q[1]
            target_pose.pose.orientation.z = q[2]
            target_pose.pose.orientation.w = q[3]

            self.arm_torso_group.set_pose_target(target_pose)

            success = self.arm_torso_group.go(wait=True)
            self.arm_torso_group.stop()
            self.arm_torso_group.clear_pose_targets()

            
            # # Get plan
            # plan = self.arm_group.plan()
             
            # success = plan[0]
            # trajectory = plan[1]
             
            # if success:
            #     rospy.loginfo("Planning successful, executing trajectory...")
            #     self.arm_group.execute(trajectory, wait=True)
            #     rospy.loginfo("Arm moved above object, starting grasp...")

            # else:
            #     rospy.logerr("Planning failed!")
                
        except Exception as e:
            rospy.logerr(f"Error moving arm: {str(e)}")

    def grasp_object(self, target):
        """
        Grasp the target object
        """
        type = self.identify_object(target)
        rospy.loginfo(f"Grasping object of type: {type}")

        # Move the arm down to grasp the object through a linear movement

                    

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