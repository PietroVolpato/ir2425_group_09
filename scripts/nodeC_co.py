import rospy
from std_msgs.msg import String
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from ir2425_group_09.msg import Detections
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class NodeC_co:
    def __init__ (self):
        rospy.init_node('nodeC_co')

        self.initial_time = rospy.Time.now()
        # Subscribe to the detected_objects topic
        rospy.Subscriber('/detected_objects', Detections, self.detection_callback)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.table_positions = [(7.8, -2.0), (7.8, -3.0)]

        self.man_pub = rospy.Publisher('/start_manipulation', PoseStamped, queue_size=10)
        
        # Initialize the PlanningSceneInterface
        self.scene = PlanningScene()
        
        # Create a publisher for the planning scene
        self.scene_pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10)
        
        rospy.sleep(1.0)

        # # Add the pickup table to the planning scene
        # pickup_table = self.create_pickup_table_collision_object()
        # self.scene.world.collision_objects.append(pickup_table)

        self.current_objects = []

        self.table = self.create_pickup_table()

    def detection_callback(self, msg):
        poses = msg.poses
        ids = msg.ids
        types = msg.types
        target = msg.target
        
        # Create a PlanningScene message
        planning_scene = PlanningScene()
        planning_scene.is_diff = True  # Set to True to update the scene

        for i in range(len(poses)):
            id = ids[i]
            pose = poses[i]
            type = types[i]
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z

            print(f"Received detection: id = {id}, type = {type}, x = {x:.2f}, y = {y:.2f}, z = {z:.2f}")
            collision_obj = self.add_collision_object(id, type, pose)
            if collision_obj:
                # Add the collision object to the planning scene
                planning_scene.world.collision_objects.append(collision_obj)
                self.current_objects.append(collision_obj)

        # Add the pickup table to the planning scene
        self.update_table()

        # Publish the planning scene
        self.scene_pub.publish(planning_scene)

        # Send to the manipulator the target object
        index = ids.index(target)
        pose_target = poses[index]
        pose_stamped_target = PoseStamped()
        pose_stamped_target.header.frame_id = "base_link"
        pose_stamped_target.pose = pose_target
        self.man_pub.publish(pose_stamped_target)

    def create_pickup_table(self):
        table_pose = PoseStamped()
        table_pose.header.frame_id = "map"
        table_pose.pose.position.x = self.table_positions[1][0]
        table_pose.pose.position.y = self.table_positions[1][1]
        table_pose.pose.position.z = 0.9

        return table_pose
    
    def update_table(self):
        if not hasattr(self, 'table'):
            rospy.logwarn("No table object available to update")
            return

        # Create new collision object
        new_table = CollisionObject()
        new_table.id = "pickup table"
        new_table.header.frame_id = "base_link"
        new_table.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.92, 0.92, 0.9]))

        try:
            # Clear previous table object
            self.scene.world.collision_objects = [obj for obj in self.scene.world.collision_objects if obj.id != "pickup table"]
            
            # Get transform with timeout
            transform = self.tfBuffer.lookup_transform("base_link", 
                                                     self.table.header.frame_id, 
                                                     rospy.Time(0),
                                                     rospy.Duration(1.0))
            
            # Create and transform pose
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose = self.table.pose
            pose_stamped.pose.position.z -= 0.9 / 2  # Adjust for table height
            pose_stamped.pose.orientation.w = 1.0
            
            transformed_pose = do_transform_pose(pose_stamped, transform)
            new_table.primitive_poses.append(transformed_pose.pose)
            new_table.operation = CollisionObject.ADD
            
            # Update scene
            self.scene.world.collision_objects.append(new_table)
            self.scene_pub.publish(self.scene)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Transform error for table: {str(e)}")
        except Exception as e:
            rospy.logerr(f"Failed to update table: {str(e)}")
    
    def add_placing_table(self):
        """
        Add the placing table to the planning scene
        """
        table = CollisionObject()
        table.id = "placing_table"
        table.header.frame_id = "map"
        table.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.95, 0.95, 0.02]))
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = self.table_positions[0][0]
        pose.pose.position.y = self.table_positions[0][1]
        pose.pose.position.z = 0.88
        table.primitive_poses.append(pose.pose)
        table.operation = CollisionObject.ADD
        return table


    def add_collision_object(self, id, type, pose):
        """
        Add a collision object to the planning scene.
        """
        obj = CollisionObject()
        obj.id = str(id)
        obj.header.frame_id = "base_link"
        obj.header.stamp = rospy.Time.now()

        if type == "cube":
            obj.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.05, 0.05, 0.05]))
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.pose = pose
            pose_stamped.pose.position.z -= 0.05 / 2
            obj.primitive_poses.append(pose_stamped.pose)
            obj.operation = CollisionObject.ADD
            return obj

        elif type == "hexagonal prism":
            obj.primitives.append(SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[0.1, 0.025]))
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.pose = pose 
            pose_stamped.pose.position.z -= 0.1 / 2
            obj.primitive_poses.append(pose_stamped.pose)
            obj.operation = CollisionObject.ADD
            return obj

        elif type == "triangular prism":
            obj.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.07, 0.05, 0.035]))
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.pose = pose
            obj.primitive_poses.append(pose_stamped.pose)
            obj.operation = CollisionObject.ADD
            return obj

        return None
    
if __name__ == '__main__':
    node = NodeC_co()
    rospy.spin()     