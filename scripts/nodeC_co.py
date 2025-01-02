import rospy
from std_msgs.msg import String
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
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
        self.table_positions = [(8.0, -1.0), (8.0, -2.0)]

        self.man_pub = rospy.Publisher('/start_manipulation', CollisionObject, queue_size=10)
        
        # Initialize the PlanningSceneInterface
        self.scene = PlanningSceneInterface()
        
        # Create a publisher for the planning scene
        self.scene_pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10)
        
        rospy.sleep(1.0)

        self.current_objects = []

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
            collision_obj = self.add_collision_object(id, type, pose)
            if collision_obj:
                # Add the collision object to the planning scene
                planning_scene.world.collision_objects.append(collision_obj)
                self.current_objects.append(collision_obj)

        # Add the pickup table to the scene
        pickup_table = self.create_pickup_table_collision_object()
        if pickup_table:
            planning_scene.world.collision_objects.append(pickup_table)

        # Publish the planning scene
        self.scene_pub.publish(planning_scene)

        # Send to the manipulator the target object
        for obj in self.current_objects:
            if obj.id == str(target):
                self.man_pub.publish(obj)

    def create_pickup_table_collision_object(self):
        """
        Create a collision object for the pickup table and return it.
        """
        table = CollisionObject()
        table.id = "pickup table"
        table.header.frame_id = "map"
        table.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.95, 0.95, 0.02]))
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = 1 #self.table_positions[1][0] 
        pose.pose.position.y = 0 #self.table_positions[1][1]
        pose.pose.position.z = 0.8
        table.primitive_poses.append(pose.pose)
        table.operation = CollisionObject.ADD
        return table
    
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
            obj.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.1, 0.1, 0.1]))
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.pose = pose
            pose_stamped.pose.position.z += 0.05
            obj.primitive_poses.append(pose_stamped.pose)
            obj.operation = CollisionObject.ADD
            return obj

        elif type == "hexagonal prism":
            obj.primitives.append(SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[0.25, 0.1]))
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.pose = pose 
            pose_stamped.pose.position.z += 0.125
            obj.primitive_poses.append(pose_stamped.pose)
            obj.operation = CollisionObject.ADD
            return obj

        elif type == "triangular prism":
            obj.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.1, 0.1, 0.05]))
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.pose = pose
            pose_stamped.pose.position.z += 0.025
            obj.primitive_poses.append(pose_stamped.pose)
            obj.operation = CollisionObject.ADD
            return obj

        return None
    
if __name__ == '__main__':
    node = NodeC_co()
    rospy.spin()     