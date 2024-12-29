import rospy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from ir2425_group_09.msg import Detections
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class NodeC:
    def __init__ (self):
        rospy.init_node('nodeC')

        # Define the subscriber to receive the detections
        rospy.Subscriber('/detected_objects', Detections, self.process_detections)

        self.collision_objects = {'1': None, '2': None, '3': None, '4': None, '5': None, '6': None, '7': None, '8': None, '9': None}
        self.scene = PlanningSceneInterface()

    def process_detections(self, msg):
        poses = msg.poses
        ids = msg.ids
        target = msg.target
        print(f"target is : {target}")
        print("received detections:")
        for i in range(len(poses)):
            id = ids[i]
            pose = poses[i]
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            w = pose.orientation.w
            print(f"id: {id}, x = {x:.3f}, y = {y:.3f}, z = {z:.3f}, w = {w:.3f}")

            # Convert pose to PoseStamped
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.pose = pose

            # Classify and create collision object
            object_type = self.classify_object(id)
            object = self.create_collision_objects(object_type, pose_stamped)
            self.collision_objects.update({str(id): object})
            print(f"Created collision object for {object_type}")
            self.scene.add_object(object)
            print(f"Added collision object to the scene: {self.collision_objects[str(id)].id}")

    def classify_object(self, tag_id):
        if tag_id in [1, 2, 3]:
            return "hexagonal prism"
        elif tag_id in [4, 5, 6]:
            return "cube"
        elif tag_id in [7, 8, 9]:
            return "triangular prism"
        else:
            return "unknown"

    def create_collision_objects(self, object_type, pose_stamped):
        object = CollisionObject()
        object.header.frame_id = "base_link"
        object.id = object_type
        
        if object_type == "cube":
            object.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.06, 0.06, 0.06]))
            object.primitive_poses.append(pose_stamped.pose)
        elif object_type == "hexagonal prism":
            object.primitives.append(SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[0.3, 0.06]))
            object.primitive_poses.append(pose_stamped.pose)
        elif object_type == "triangular prism":
            object.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.06, 0.04, 0.08]))
            object.primitive_poses.append(pose_stamped.pose)
        
        object.operation = CollisionObject.ADD
        return object

    def run(self):
        # Define the collision object of the table
        table = CollisionObject()
        # table.id = "table"
        # table.header.frame_id = "base_link"
        # table.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[1.0, 1.0, 1.0]))
        # pose = PoseStamped()
        # pose.header.frame_id = "base_link"
        # pose.pose.position.x = 
        # pose.pose.position.y = 
        # pose.pose.position.z = 
        # pose.pose.orientation.w = 
        # table.primitive_poses.append(pose)
        # table.operation = CollisionObject.ADD
        self.scene.add_object(table)
        rospy.loginfo("Table added to the scene.")

        rospy.spin()

if __name__ == '__main__':
    try:
        node = NodeC()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node C stopped.")