import rospy
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import PlanningSceneInterface, MoveGroupCommander
from std_msgs.msg import String, Int32
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

        rospy.Subscriber('/picking_routine_feedback', Int32, self.get_target_id)

        # to notify nodeA_navigation outcome of placing routine
        self.feedback_pub = rospy.Publisher('/placing_routine_feedback', String, queue_size=10)

        # Initialize actionlib client
        #self.nav_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        #rospy.loginfo("Waiting for move_base action server...")
        #self.nav_client.wait_for_server()
        #rospy.loginfo("Connected to move_base action server.")

        rospy.wait_for_service('/link_attacher_node/detach', timeout=5.0)
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)

        self.scene = PlanningSceneInterface()

        self.arm_group = MoveGroupCommander("arm") # Initialize the MoveIt commander for the arm
        self.arm_torso_group = MoveGroupCommander("arm_torso")  # Initialize the MoveIt commander for arm_torso
        self.gripper_group = MoveGroupCommander("gripper")  # Initialize the MoveIt commander for the gripper

        self.arm_torso_group.set_max_velocity_scaling_factor(0.5)
        self.arm_torso_group.set_max_acceleration_scaling_factor(0.5)
        self.arm_torso_group.set_planning_time(10.0) 

        self.default_config = self.arm_torso_group.get_current_joint_values()

        self.target_id = None

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

        # place the object on the target point
        self.place_object(x, y, z, object_height)

    def get_target_id(self, msg):
        """
        Get the target id of the object to place
        """
        self.target_id = msg.data

    def place_object(self, x, y, z, object_height):
        """
        Place the object on the target point
        """
        try:
            # move the gripper above the target point
            target_pose = Pose()
            target_pose.position.x = x
            target_pose.position.y = y
            target_pose.position.z = z

            # align the gripper vertically
            self.align_gripper_vertically(target_pose)
            rospy.loginfo("Arm positioned above the target point")

            # open the gripper
            self.open_gripper()
            rospy.loginfo("Gripper opened")

            # detach object from gripper
            self.detach_object_from_gripper()
            rospy.loginfo("Object detached from gripper")

            # move the arm to the default configuration
            self.move_to_default_config()

            # remove collision object from planning scene
            self.remove_collision_object("placement_table")

            # notify nodeA_navigation that placing routine is completed
            self.feedback_pub.publish("placing_routine_completed")
        except Exception as e:
            rospy.logerr(f"Error placing object: {str(e)}")

    def align_gripper_vertically(self, target_pose, z_offset = 0.3):
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

        rospy.loginfo("Gripper closed")

    def open_gripper(self, opening=0.08):
        """
        Open the gripper to release the object.
        """
        # Define the joint goal to open the gripper
        joint_goal = self.gripper_group.get_current_joint_values()
        joint_goal[0] = opening / 2
        joint_goal[1] = opening / 2

        # Plan and execute the motion
        self.gripper_group.go(joint_goal, wait=True)
        self.gripper_group.stop()

        rospy.loginfo("Gripper opened")

    def detach_object_from_gripper(self, gripper_link="tiago::gripper_left_finger_link"):
        """
        Detach the target object from the gripper using the Gazebo_ros_link_attacher plugin.
        """
        model_name = self.model_names[self.target_id]
        link_name = f"{model_name}::{model_name}_link"
    
        try:    
            # Create the detach request
            req = AttachRequest()
            req.model_name_1 = "tiago"  # Robot model name
            req.link_name_1 = gripper_link
            req.model_name_2 = model_name
            req.link_name_2 = link_name

            # Call the service
            self.attach_srv.call(req)
            rospy.loginfo(f"Object {self.target_id} ({model_name}) detached from gripper")

        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to detach object: {str(e)}")
        except rospy.ROSException as e:
            rospy.logerr(f"Service call failed: {str(e)}")

    def move_to_default_config (self):
        """
        Move the arm to the default configuration
        """
        self.arm_torso_group.go(self.default_config, wait=True)
        self.arm_torso_group.stop()

        rospy.loginfo("Arm moved to default configuration")

if __name__ == '__main__':
    try:
        node = nodeA_placing_routine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass