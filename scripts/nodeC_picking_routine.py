import rospy
from geometry_msgs.msg import PoseStamped
from moveit_commander import PlanningSceneInterface, MoveGroupCommander
from std_msgs.msg import String
from std_msgs.msg import Int32
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
import math
from tf.transformations import quaternion_from_euler
from ir2425_group_09.msg import TargetObject  # custom message
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class nodeC_picking_routine:
    def __init__ (self):
        rospy.init_node('nodeC_picking_routine')

        # Subscribe to the picking routine topic
        rospy.Subscriber('/picking_routine', TargetObject, self.picking_routine)

        # Publish the trajectory to move the torso
        self.torso_pub = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=10) # to move the torso

        # Publish the trajectory to move the arm
        self.arm_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10) # to move the arm

        # Publish the feedback of the picking routine
        self.feedback_pub = rospy.Publisher('/picking_routine_feedback', Int32, queue_size=10)

        # Publisher to ask the planning scene to add the table
        self.table_pub = rospy.Publisher('/table_co', String, queue_size=10) 

        # Wait for the service of the link attacher to be available
        rospy.wait_for_service('/link_attacher_node/attach', timeout=5.0)
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)

        # Initialize the planning scene interface
        self.scene = PlanningSceneInterface()

        self.arm_group = MoveGroupCommander("arm") # Initialize the MoveIt commander for the arm
        self.arm_torso_group = MoveGroupCommander("arm_torso")  # Initialize the MoveIt commander for arm_torso
        self.gripper_group = MoveGroupCommander("gripper")  # Initialize the MoveIt commander for the gripper

        # Set the maximum velocity and acceleration scaling factors
        self.arm_torso_group.set_max_velocity_scaling_factor(0.5)
        self.arm_torso_group.set_max_acceleration_scaling_factor(0.5)
        self.arm_torso_group.set_planning_time(10.0)  

        # Object list where the key is the object id and the value is the object name
        self.object_list = {1 : 'hexagonal prism', 2: 'hexagonal prism', 3 : 'hexagonal prism',
                            4: 'cube', 5: 'cube', 6 : 'cube',
                            7: 'triangular prism', 8 : 'triangular prism', 9 : 'triangular prism'}
        
        # Gripper length, used to calculate the z offset for the gripper to place it above the object
        self. gripper_length = 0.226

        # Object heights where the key is the object id and the value is the object height
        self.object_heights = { 1 : 0.1, 2 : 0.1, 3 : 0.1,
                                4 : 0.05, 5 : 0.05, 6 : 0.05,
                                7 : 0.035, 8 : 0.035, 9 : 0.035}

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
        
        # Wait for the robot to be ready for moving to the initial configuration
        rospy.sleep(10.0)
        self.initial_config()

    def initial_config(self):
        """
        Move the arm to initial configuration. 
        """
        # Move the torso to the initial position
        position = 0.35
        duration = 2.0
        traj = JointTrajectory()
        traj.joint_names = ["torso_lift_joint"]

        # Create a point
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = rospy.Duration(duration)

        # Append the point to the trajectory
        traj.points.append(point)

        # Publish the trajectory
        self.torso_pub.publish(traj)

    def picking_routine(self, msg):
        """
        Perform the picking routine for the target object.
        The routine works as follows:
            1. Raise the torso to the initial position if it is not already there.
            2. Move the arm to the intermediate configuration. This position is used to avoid collision with the table and consists in the arm that is raised up in Tiago's side.
            3. Move the arm above the target object. The gripper is placed forced to point downwards and above the object of a given height that is different for the objects.
            4. Remove the collision object of the picking table.
            5. Move the arm down to the object. For the cube and the hexagoanl prism, the gripper is placed half the object height, for the triangular prism, the gripper is placed 1.9 cm above the object.
            6. Remove the collision object of the target object.
            7. Close the gripper until contact with the object.
            8. Attach the object to the gripper.
            9. Move the arm up after picking the object. 
            10. Re create the collision object of the picking table.
            11. Move the arm to safe configuration. The position of the armn is the same as Tiago initial configuration at the beginning of the simuilation, i.e. with the arm close to tis body.
            12. Remove all objects from the planning scene.
            13. Publish the feedback of the picking routine.
        """
        # Move the arm to the initial configuration
        self.initial_config()

        # Get the target pose and id
        target_pose = msg.pose
        target_id = msg.id

        # Define the z offset for the gripper to place it above the object
        z_above_object = 0.4 - self.object_heights[target_id]  # z offset of the position of the arm above the object
        if target_id in [1, 2, 3, 4, 5, 6]:
            z_on_object = self.gripper_length - self.object_heights[target_id] / 2 # place the gripper half the object height above the table
        else:
            z_on_object = self.gripper_length + 0.019  # if it is a triangular prism, place the gripper 1.9 cm above the object
            target_pose.position.x += 0.01 # center the gripper on the object
            target_pose.position.y += 0.01 # center the gripper on the object

        rospy.sleep(1)  # give time planning scene to initialize
        rospy.loginfo(f"Starting PICKING ROUTINE. Target is obj {target_id} ({self.object_list[target_id]})")
        
        # Move the arm to the intermediate configuration
        self.intermediate_pose()

        # Move the arm above the target object
        self.align_gripper_vertically(target_pose, z_above_object) 
        rospy.loginfo(f"Arm positioned above the target object")

        # Remove the collision object of the picking table
        self.remove_collision_object("pickup_table")

        # Move the arm down to the object
        self.align_gripper_vertically(target_pose, z_on_object)
        rospy.loginfo("Gripper in position, ready to close.") 

        # Remove the collision object of the target object
        self.remove_collision_object(target_id)

        # Close the gripper until contact
        self.close_gripper_until_contact()
        rospy.loginfo(f"Gripper closed")

        # Attach the object to the gripper
        self.attach_object_to_gripper(target_id)
        rospy.loginfo(f"Attached object to the gripper.")

        # Move the arm up after picking the object
        self.align_gripper_vertically(target_pose, z_above_object)
        
        # Re create the collision object of the picking table
        self.table_pub.publish(String(data="picking"))

        # Move the arm to the safe configuration after picking the object
        rospy.loginfo("Moving arm to safe pose")
        self.move_to_safe_configuration()

        # Remove all objects from the planning scene
        self.remove_all_objects()

        # Publish the feedback of the picking routine
        self.feedback_pub.publish(Int32(data=target_id))

        rospy.loginfo(f"Object lifted, PICKUP ROUTINE COMPLETED")

    def align_gripper_vertically(self, target_pose, z_offset):
        """
        Move the arm to a position above the target object, with the gripper pointing downwards.
        """
        try:

            # Create a PoseStamped message
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "base_link"

            # Set the position of the target object with the z offset
            goal_pose.pose.position.x = target_pose.position.x
            goal_pose.pose.position.y = target_pose.position.y
            goal_pose.pose.position.z = target_pose.position.z + z_offset

            # Clockwise rotation around y axis to point gripper downward
            q = quaternion_from_euler(0, math.pi/2, 0)
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
                    break

            except Exception as e:
                rospy.logerr(f"Error during gripper closing: {str(e)}")
                break
  
    # there are 2 links of the gripper: tiago::gripper_left_finger_link and tiago::gripper_right_finger_link
    def attach_object_to_gripper(self, target_id, gripper_link="tiago::gripper_left_finger_link"):  
        """
        Attach the target object to the gripper using the Gazebo_ros_link_attacher plugin.
        """

        # Get the model name and link name of the target object
        model_name = self.model_names[target_id]  # name of the object in gazebo
        link_name = f"{model_name}::{model_name}_link" # name of the link of the object in gazebo
        
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
        
        # Move the arm to the intermediate configuration
        try:
            self.arm_torso_group.set_joint_value_target(configuration)
            self.arm_torso_group.go(wait=True)
            self.arm_torso_group.stop()

        except Exception as e:
            rospy.logerr(f"Failed to move arm to default configuration: {e}")

    def move_to_safe_configuration(self):
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
        
        # Move the arm to the configuration
        try:
            self.arm_torso_group.set_joint_value_target(configuration_2)
            self.arm_torso_group.go(wait=True)
            self.arm_torso_group.stop()

        except Exception as e:
            rospy.logerr(f"Failed to move arm to default configuration: {e}")

    def remove_all_objects(self):
        """
        Remove all objects from the planning scene.
        """
        # Remove all objects from the planning scene
        scene = PlanningSceneInterface()
        scene.remove_world_object()
        rospy.sleep(1.0) # wait for scene update

if __name__ == '__main__':
    try:
        node = nodeC_picking_routine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass