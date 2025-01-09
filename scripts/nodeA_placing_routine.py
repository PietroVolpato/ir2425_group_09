import rospy
from geometry_msgs.msg import PoseStamped
from moveit_commander import PlanningSceneInterface, MoveGroupCommander
from std_msgs.msg import String
import tf2_ros
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from math import pi
from tf.transformations import quaternion_from_euler
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from ir2425_group_09.msg import PlacingMessage  # custom message

class nodeA_placing_routine:
    def __init__ (self):
        rospy.init_node('nodeA_placing_routine')

        # obtain coordinates of placing point
        rospy.Subscriber('/placing_routine', PlacingMessage, self.placing_routine)

        # to notify nodeA_navigation outcome of placing routine
        self.feedback_pub = rospy.Publisher('/placing_routine_feedback', String, queue_size=10)

        # Initialize actionlib client
        #self.nav_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        #rospy.loginfo("Waiting for move_base action server...")
        #self.nav_client.wait_for_server()
        #rospy.loginfo("Connected to move_base action server.")

        rospy.wait_for_service('/link_attacher_node/attach', timeout=5.0)
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)

        self.scene = PlanningSceneInterface()

        self.arm_group = MoveGroupCommander("arm") # Initialize the MoveIt commander for the arm
        self.arm_torso_group = MoveGroupCommander("arm_torso")  # Initialize the MoveIt commander for arm_torso
        self.gripper_group = MoveGroupCommander("gripper")  # Initialize the MoveIt commander for the gripper

        self.arm_torso_group.set_max_velocity_scaling_factor(0.5)
        self.arm_torso_group.set_max_acceleration_scaling_factor(0.5)
        self.arm_torso_group.set_planning_time(10.0)  

        self.object_list = {1 : 'hexagonal prism', 2: 'hexagonal prism', 3 : 'hexagonal prism',
                            4: 'cube', 5: 'cube', 6 : 'cube',
                            7: 'traingular prism', 8 : 'traingular prism', 9 : 'traingular prism'}

        # map from object ids to model names in gazebo 
        self.model_names = {
            1 : "Hexagon",
            2 : "Hexagon_2",
            3 : "Hexagon_3",
            4 : "cube",
            5 : "cube_5",
            6 : "cube_6",
            7 : "Triangle",
            8 : "Triangle_8",
            9 : "Triangle_9"
        }
        
    def placing_routine(self, msg):
        """
        placing routine
        """
        # target placing point coordinates in base link
        x = msg.x
        y = msg.y
        z = msg.z
        object_height = msg.object_height
        
        rospy.sleep(1)  # give time planning scene to initialize
        rospy.loginfo(f"starting PLACING ROUTINE. Selected placement point: ({x:.3f},{y:.3f},{z:.3f})")
        return

    def align_gripper_vertically(self, target_pose, z_offset = 0.35):
        """
        This function place the gripper on the following pose:
            - x and y are the same of the target pose
            - z is the target pose + z_offset (with sign)
            - the gripper points downwards
        
        If the current pose of the gripper and and the target pose have same x and y, then it is performed a vertical movement along
        z axis by z_offset meters.
        """
        try:

            # create a pose 35 cm above target, with gripper pointing downwards
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "base_link"

            goal_pose.pose.position.x = target_pose.position.x
            goal_pose.pose.position.y = target_pose.position.y
            goal_pose.pose.position.z = target_pose.position.z + z_offset

            q = quaternion_from_euler(0, pi/2, 0)  # clockwise rotation around y axis to point gripper downward
            # Set the orientation
            goal_pose.pose.orientation.x = q[0]
            goal_pose.pose.orientation.y = q[1]
            goal_pose.pose.orientation.z = q[2]
            goal_pose.pose.orientation.w = q[3]

            # Set the pose target for the arm
            self.arm_torso_group.set_pose_target(goal_pose)

            # Plan and execute the motion
            success = self.arm_torso_group.go(wait=True)
            self.arm_torso_group.stop()
            self.arm_torso_group.clear_pose_targets()

            if not success:
                rospy.logerr("Failed to move arm above object.")

        except Exception as e:
            raise Exception(f"Error moving arm: {str(e)}")

    def remove_collision_object(self, object_name):
        """
        Remove a collision object from the planning scene.

        Args:
            object_name (str): Name of the object to remove.
        """
        scene = PlanningSceneInterface()

        # Remove the object by name
        scene.remove_world_object(str(object_name))
        rospy.sleep(1.0) # wait for scene update
    
    def close_gripper(self, opening):
        """
        Close the gripper to grasp the object.
        """
        # Define the joint goal to close the gripper
        joint_goal = self.gripper_group.get_current_joint_values()
        #print(f"current joint values: {joint_goal}")
        # to have 'opening' meters of space between fingers, each finger joint is set to opening/2
        joint_goal[0] = opening/2  # how wide is left finger joint 
        joint_goal[1] = opening/2 #  how wide is right finger joint

        # Plan and execute the motion
        self.gripper_group.go(joint_goal, wait=True)
        self.gripper_group.stop()

        rospy.loginfo("Gripper closed")
    
    # there are 2 links of the gripper: tiago::gripper_left_finger_link and tiago::gripper_right_finger_link
    def attach_object_to_gripper(self, target_id, gripper_link="tiago::gripper_left_finger_link"):  
        """
        Attach the target object to the gripper using the Gazebo_ros_link_attacher plugin.
        """

        model_name = self.model_names[target_id]   # name of the object model in gazebo
        link_name = f"{model_name}::{model_name}_link"   # name of the link of the object in gazebo
        print(f"model name: {model_name}")
        print(f"link name: {link_name}")
        try:    
            # Create the attach request
            req = AttachRequest()
            req.model_name_1 = "tiago"  # Robot model name
            req.link_name_1 = gripper_link
            req.model_name_2 = model_name
            req.link_name_2 = link_name

            # Call the service
            self.attach_srv.call(req)

        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to attach object: {str(e)}")
        except rospy.ROSException as e:
            rospy.logerr(f"Service call failed: {str(e)}") 

if __name__ == '__main__':
    try:
        node = nodeA_placing_routine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass