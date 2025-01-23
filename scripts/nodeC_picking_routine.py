import rospy
from geometry_msgs.msg import PoseStamped
from moveit_commander import PlanningSceneInterface, MoveGroupCommander
from std_msgs.msg import String
from std_msgs.msg import Int32
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
import math
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from ir2425_group_09.msg import TargetObject  # custom message
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class nodeC_picking_routine:
    def __init__ (self):
        rospy.init_node('nodeC_picking_routine')

        # Subscribe to the nav_goal topic
        rospy.Subscriber('/picking_routine', TargetObject, self.picking_routine)
        self.torso_pub = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=10) # to move the torso

        self.arm_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10) # to move the arm

        self.feedback_pub = rospy.Publisher('/picking_routine_feedback', Int32, queue_size=10) # to move the camera angle

        self.table_pub = rospy.Publisher('/table_co', String, queue_size=10) 

        # Initialize actionlib client
        #self.nav_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        #rospy.loginfo("Waiting for move_base action server...")
        #self.nav_client.wait_for_server()
        #rospy.loginfo("Connected to move_base action server.")

        rospy.wait_for_service('/link_attacher_node/attach', timeout=5.0)
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.scene = PlanningSceneInterface()

        self.arm_group = MoveGroupCommander("arm") # Initialize the MoveIt commander for the arm
        self.arm_torso_group = MoveGroupCommander("arm_torso")  # Initialize the MoveIt commander for arm_torso
        self.gripper_group = MoveGroupCommander("gripper")  # Initialize the MoveIt commander for the gripper

        self.arm_torso_group.set_max_velocity_scaling_factor(0.5)
        self.arm_torso_group.set_max_acceleration_scaling_factor(0.5)
        self.arm_torso_group.set_planning_time(10.0)  

        self.object_list = {1 : 'hexagonal prism', 2: 'hexagonal prism', 3 : 'hexagonal prism',
                            4: 'cube', 5: 'cube', 6 : 'cube',
                            7: 'triangular prism', 8 : 'triangular prism', 9 : 'triangular prism'}
        
        self. gripper_length = 0.226

        self.object_heights = { 1 : 0.1, 2 : 0.1, 3 : 0.1,
                                4 : 0.05, 5 : 0.05, 6 : 0.05,
                                7 : 0.035, 8 : 0.035, 9 : 0.035}

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
        
        rospy.sleep(10.0)
        self.initial_config()

    def initial_config(self):
        """
        Move the arm to initial configuration with proper state handling
        """
        rospy.loginfo("Moving arm to initial configuration")
        
        position = 0.35
        duration = 2.0
        traj = JointTrajectory()
        traj.joint_names = ["torso_lift_joint"]

        # Definisci un punto di traiettoria
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = rospy.Duration(duration)

        # Aggiungi il punto al messaggio
        traj.points.append(point)

        self.torso_pub.publish(traj)

    def picking_routine(self, msg):
        """
        Start the manipulation process
        """
        print("Moving arm to initial configuration")
        self.initial_config()

        target_pose = msg.pose
        target_id = msg.id

        z_above_object = 0.4 - self.object_heights[target_id]  # z offset of the position of the arm above the object
        if target_id in [1, 2, 3, 4, 5, 6]:
            z_on_object = self.gripper_length - self.object_heights[target_id] / 2
        else:
            z_on_object = self.gripper_length + 0.021  # place the gripper about on half height of the object
            target_pose.position.x += 0.01
            target_pose.position.y += 0.01

        rospy.sleep(1)  # give time planning scene to initialize
        rospy.loginfo(f"Starting PICKING ROUTINE. Target is obj {target_id} ({self.object_list[target_id]})")
        
        yaw = 0 #self.correct_gripper_orientation(target_pose)

        self.intermediate_pose(yaw=yaw)  # intermediate pose to raise the arm
        self.align_gripper_vertically(target_pose, z_above_object) # place arm 35cm above object
        rospy.loginfo(f"Arm positioned above the target object")

        # Remove the collision object of the picking table
        self.remove_collision_object("pickup_table")

        # grasp pose 20 cm above target because the frame is above the gripper fingers, which are long slightly less than 0.2
        self.align_gripper_vertically(target_pose, z_on_object) # gripper surrounds the object
        rospy.loginfo("Gripper in position, ready to close.") 

        self.remove_collision_object(target_id)
        rospy.loginfo(f"Removed collision object of the target object ({target_id})")

        self.close_gripper_until_contact()
        rospy.loginfo(f"Gripper closed")

        self.attach_object_to_gripper(target_id)
        rospy.loginfo(f"Attached object {target_id} ({self.model_names[target_id]}) to the gripper.")

        self.align_gripper_vertically(target_pose, z_above_object)  # lift object
        rospy.loginfo(f"Object lifted, PICKUP ROUTINE COMPLETED")

        # Re create the collision object of the picking table
        self.table_pub.publish(String(data="picking"))  # publish the table position for the planning scene

        self.move_to_safe_configuration()
        rospy.loginfo("Arm moved to safe configuration")

        self.remove_all_objects()

        self.feedback_pub.publish(Int32(data=target_id))

    def align_gripper_vertically(self, target_pose, z_offset):
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

            q = quaternion_from_euler(0, math.pi/2, 0)  # clockwise rotation around y axis to point gripper downward
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
        
    def correct_gripper_orientation(self, target_pose):
        """Corrects gripper orientation based on target pose.
        Args:
            target_pose: PoseStamped containing desired orientation
        """
        # Convert the pose from base_link to camera
        
        try:
            transform = self.tf_buffer.lookup_transform("base_link", "xtion_rgb_optical_frame", rospy.Time(0))
            target_pose_camera = do_transform_pose(target_pose, transform)
        except Exception as e:
            raise Exception(f"Failed to transform pose: {str(e)}")
        
        # Extract quaternion and convert to euler angles
        q = target_pose_camera.pose.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw = euler[2]
        #     # Extract quaternion and convert to euler angles
        # q = target_pose.orientation
        # euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        print(f"Yaw: {yaw}")
        return yaw
        
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
    
    def close_gripper_until_contact(self):
        """
        Close the gripper fingers until they make contact with the object, accounting for asymmetry.
        """
        # Get the current joint values
        joint_goal = self.gripper_group.get_current_joint_values()

        # Define the minimum allowed opening for the gripper
        min_opening = 0.02  # almost fully closed
        closing_step = 0.005  # Increment per step
        max_closing_attempts = 50  # Maximum number of steps to close the gripper

        for attempt in range(max_closing_attempts):
            try:
                # Check current joint positions for both fingers
                current_joints = self.gripper_group.get_current_joint_values()
                left_finger_joint = current_joints[0]
                right_finger_joint = current_joints[1]

                # Gradually reduce joint values for both fingers
                if left_finger_joint > min_opening:
                    joint_goal[0] -= closing_step / 2  # Left finger
                    joint_goal[0] = max(joint_goal[0], min_opening)

                if right_finger_joint > min_opening:
                    joint_goal[1] -= closing_step / 2  # Right finger
                    joint_goal[1] = max(joint_goal[1], min_opening)

                # Plan and execute the motion
                success = self.gripper_group.go(joint_goal, wait=True)
                self.gripper_group.stop()

                if not success:
                    rospy.logwarn(f"Failed to move the gripper on attempt {attempt}. Stopping.")
                    break

                # Check if either finger has stopped moving, indicating contact
                updated_joints = self.gripper_group.get_current_joint_values()
                if abs(updated_joints[0] - left_finger_joint) < closing_step / 4 and abs(updated_joints[1] - right_finger_joint) < closing_step / 4:
                    rospy.loginfo("Gripper fingers stopped moving; assumed object contact.")
                    break

            except Exception as e:
                rospy.logerr(f"Error during gripper closing: {str(e)}")
                break
  
    # there are 2 links of the gripper: tiago::gripper_left_finger_link and tiago::gripper_right_finger_link
    def attach_object_to_gripper(self, target_id, gripper_link="tiago::gripper_left_finger_link"):  
        """
        Attach the target object to the gripper using the Gazebo_ros_link_attacher plugin.
        """

        model_name = self.model_names[target_id]   # name of the object model in gazebo
        link_name = f"{model_name}::{model_name}_link"   # name of the link of the object in gazebo
        #print(f"model name: {model_name}")
        #print(f"link name: {link_name}")
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
    
    def intermediate_pose(self, yaw = 0):
        """
        Move the arm to the intermediate configuration
        """
        configuration = {
                'torso_lift_joint': 0.35,
                'arm_1_joint': 0.1,
                'arm_2_joint': 0,
                'arm_3_joint': -0.2,
                'arm_4_joint': 0,
                'arm_5_joint': -1.57,
                'arm_6_joint': 0.8,
                'arm_7_joint': yaw
        }
        
        try:
            self.arm_torso_group.set_joint_value_target(configuration)
            self.arm_torso_group.go(wait=True)
            self.arm_torso_group.stop()

        except Exception as e:
            rospy.logerr(f"Failed to move arm to default configuration: {e}")

        rospy.loginfo("Arm moved to default configuration")  


    def move_to_safe_configuration(self):
        """
        Move the arm to the default configuration
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

        rospy.loginfo("Arm moved to default configuration")   

    def remove_all_objects(self):
        """
        Remove all objects from the planning scene.
        """
        scene = PlanningSceneInterface()
        scene.remove_world_object()
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        node = nodeC_picking_routine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass