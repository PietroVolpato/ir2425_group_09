import rospy
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import PlanningSceneInterface, MoveGroupCommander
from std_msgs.msg import String, Int32
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from math import pi
from tf.transformations import quaternion_from_euler
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from ir2425_group_09.msg import PlacingMessage  # custom message

class nodeA_placing_routine:
    def __init__ (self):
        rospy.init_node('nodeA_placing_routine')

        # Subscribe to the placing routine topic
        rospy.Subscriber('/placing_routine', PlacingMessage, self.placing_routine)

        # Subscribe to the picking routine feedback topic
        rospy.Subscriber('/picking_routine_feedback', Int32, self.get_target_id)

        # Publish to the table_co topic
        self.table_pub = rospy.Publisher('/table_co', String, queue_size=10)

        # Publish to the placing routine feedback topic
        self.feedback_pub = rospy.Publisher('/placing_routine_feedback', String, queue_size=10)

        # Wait for the attach and detach services
        rospy.wait_for_service('/link_attacher_node/detach', timeout=5.0)
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)

        # Initialize the planning scene interface
        self.scene = PlanningSceneInterface()

        # Initialize the MoveIt commander for the arm and gripper
        self.arm_group = MoveGroupCommander("arm") # Initialize the MoveIt commander for the arm
        self.arm_torso_group = MoveGroupCommander("arm_torso")  # Initialize the MoveIt commander for arm_torso
        self.gripper_group = MoveGroupCommander("gripper")  # Initialize the MoveIt commander for the gripper

        # Set the planning time and scaling factors
        self.arm_torso_group.set_max_velocity_scaling_factor(0.5)
        self.arm_torso_group.set_max_acceleration_scaling_factor(0.5)
        self.arm_torso_group.set_planning_time(10.0) 

        # Target id of the object to place
        self.target_id = None

        # Object list where the key is the object id and the value is the object name
        self.object_list = {1 : 'hexagonal prism', 2: 'hexagonal prism', 3 : 'hexagonal prism',
                            4: 'cube', 5: 'cube', 6 : 'cube',
                            7: 'traingular prism', 8 : 'traingular prism', 9 : 'traingular prism'}

        # Map from object ids to model names in gazebo 
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

        # Gripper length
        self.gripper_length = 0.226
        
    def placing_routine(self, msg):
        """
        Callback function for the placing routine topic
        """
        # Target placing point coordinates in base link
        x = msg.x
        y = msg.y
        z = msg.z
        object_height = msg.object_height
        
        rospy.sleep(1)  # give time planning scene to initialize
        rospy.loginfo(f"starting PLACING ROUTINE. Selected placement point: ({x:.3f},{y:.3f},{z:.3f})")

        # Place the object on the target point
        self.place_object(x, y, z, object_height)

    def get_target_id(self, msg):
        """
        Get the target id of the object to place
        """
        self.target_id = msg.data

    def place_object(self, x, y, z, object_height):
        """
        Perform the placing routine for the object.
        The routine works as follows:
            1. Move the arm to the intermediate configuration. This position is used to avoid collision with the table and consists in the arm that is raised up in Tiago's side.
            2. Move the arm above the point calculated in the nodeA. The gripper is placed forced to point downwards and 31 cm above the target point.
            3. Remove the collision object of the placing table.
            4. Move the arm down to avoid dropping the object. For the cube and the hexagoanl prism, the gripper is placed half the object height, for the triangular prism, the gripper is placed at the object height.
            5. Detach the object from the gripper.
            6. Open the gripper.
            7. Raise the arm above the object.
            8. Re create the collision object of the placing table.
            9. Move the arm to safe configuration. The position of the armn is the same as Tiago initial configuration at the beginning of the simuilation, i.e. with the arm close to tis body.
            10. Publish the feedback of the picking routine.
        """
        try:
            # Move the gripper above the target point
            target_pose = Pose()
            target_pose.position.x = x
            target_pose.position.y = y
            target_pose.position.z = z

            # z offset for the gripper to be above the target point
            z_offset = 0.31 + object_height / 2 # 31 cm plus half of the object height above the target point

            # Second point for avoiding dropping the object on the table. 
            if self.target_id in [1, 2, 3, 4, 5, 6]:
                z_on_object = self.gripper_length + object_height / 2  # if it is a cube or hexagonal prism go down until the gripper is at half of the object height
            else:
                z_on_object = self.gripper_length + object_height  # if it is a triangular prism go down until the gripper is at the object height

            # Move the arm to the intermediate pose
            self.intermediate_pose()

            # Move the arm above the target point
            self.align_gripper_vertically(target_pose, z_offset)
            rospy.loginfo("Arm positioned above the target point")

            # Remove the collision object of the table
            self.remove_collision_object("placement_table")

            # Go down again to avoid dropping the object on the table
            self.align_gripper_vertically(target_pose, z_on_object)
            rospy.loginfo("Object placed on the target point")

            # Detach object from gripper
            self.detach_object_from_gripper()

            rospy.sleep(1.5) # wait for the object to detach

            # Open the gripper
            self.open_gripper()
            rospy.loginfo("Gripper opened")

            rospy.sleep(1.5) # wait for the gripper to open

            # After placing the object, move the arm above the object
            self.align_gripper_vertically(target_pose, z_offset)

            # Re create the collision object of the table
            self.table_pub.publish("placing")

            # move the arm to the safe configuration
            rospy.loginfo("Moving arm to safe configuration")
            self.move_to_safe_configuration()

            # Remove collision object from planning scene
            self.remove_collision_object("placement_table")
            self.remove_collision_object("pickup_table")

            # Notify nodeA_navigation that placing routine is completed
            self.feedback_pub.publish("placing_routine_completed")

            rospy.loginfo("PLACING ROUTINE COMPLETED")
        except Exception as e:
            rospy.logerr(f"Error placing object: {str(e)}")

    def align_gripper_vertically(self, target_pose, z_offset):
        """
        Move the arm to a position above the target point in the table, with the gripper pointing downwards.
        """
        try:

            # Set the target pose for the arm
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

    @staticmethod
    def remove_collision_object(object_name):
        """
        Remove a collision object from the planning scene.

        Args:
            object_name (str): Name of the object to remove.
        """
        scene = PlanningSceneInterface()

        # Remove the object by name
        scene.remove_world_object(str(object_name))

    def open_gripper(self, opening=0.088):
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

    def detach_object_from_gripper(self, gripper_link="tiago::gripper_left_finger_link"):
        """
        Detach the target object from the gripper using the Gazebo_ros_link_attacher plugin.
        """
        # Get the model name and link name of the object
        model_name = self.model_names[self.target_id] # name of the object in gazebo
        link_name = f"{model_name}::{model_name}_link" # link name of the object in gazebo
    
        try:    
            # Create the detach request
            req = AttachRequest()
            req.model_name_1 = "tiago"  # Robot model name
            req.link_name_1 = gripper_link
            req.model_name_2 = model_name
            req.link_name_2 = link_name

            # Call the service
            self.attach_srv.call(req)
            rospy.loginfo(f"Object detached from gripper")

        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to detach object: {str(e)}")
        except rospy.ROSException as e:
            rospy.logerr(f"Service call failed: {str(e)}")

    def move_to_safe_configuration (self):
        """
        Move the arm to the default configuration. This position is used to avoid collision with all the objects in the scene such as walls and tables while Tiago is moving around.
        The position is defined by the following joint values:
            -torso_lift_joint: 0.35
            -arm_1_joint: 0.2
            -arm_2_joint: -1.3
            -arm_3_joint: -0.2
            -arm_4_joint: 1.94
            -arm_5_joint: -1.57
            -arm_6_joint: 1.368
            -arm_7_joint: 0
        This values have been obtained by checking in the RViz interface the joint values of the arm when it is in the desired position.
        """
        self.intermediate_pose()

        configuration_2 = {
                'torso_lift_joint': 0.35,
                'arm_1_joint': 0.2,
                'arm_2_joint': -1.3,
                'arm_3_joint': -0.2,
                'arm_4_joint': 1.94,
                'arm_5_joint': -1.57,
                'arm_6_joint': 1.368,
                'arm_7_joint': 0
                }
        try:
            self.arm_torso_group.set_joint_value_target(configuration_2)
            self.arm_torso_group.go(wait=True)
            self.arm_torso_group.stop()
        except Exception as e:
            rospy.logerr(f"Failed to move arm to default configuration: {e}")

    def intermediate_pose(self):
        """
        Move the arm to the intermediate configuration. This position is used to avoid collision with the table and consists in the arm that is raised up in Tiago's side.
        The position is defined by the following joint values:
            -torso_lift_joint: 0.35
            -arm_1_joint: 0.1
            -arm_2_joint: 0
            -arm_3_joint: -0.2
            -arm_4_joint: 0
            -arm_5_joint: -1.57
            -arm_6_joint: 0.8
            -arm_7_joint: 0
        This values have been obtained by checking in the RViz interface the joint values of the arm when it is in the desired position.
        """
        configuration = {
                'torso_lift_joint': 0.35,
                'arm_1_joint': 0.1,
                'arm_2_joint': 0,
                'arm_3_joint': -0.2,
                'arm_4_joint': 0,
                'arm_5_joint': -1.57,
                'arm_6_joint': 0.8,
                'arm_7_joint': 0
        }
        
        try:
            self.arm_torso_group.set_joint_value_target(configuration)
            self.arm_torso_group.go(wait=True)
            self.arm_torso_group.stop()

        except Exception as e:
            rospy.logerr(f"Failed to move arm to default configuration: {e}")

if __name__ == '__main__':
    try:
        node = nodeA_placing_routine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
