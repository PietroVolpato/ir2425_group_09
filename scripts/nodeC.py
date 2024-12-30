#!/usr/bin/env python3

# ATTENZIONE QUESTO CODICE NON E' SCRITTO DA ME E NON FUNZIONA
# PERSISTE IL PROBLEMA CHE I COLLISION OBJECT NON SONO STATICI MA SI MUOVONO
import rospy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from ir2425_group_09.msg import Detections
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class NodeC:
    def __init__(self):
        rospy.init_node('nodeC')
        
        # Initialize the planning scene interface
        self.scene = PlanningSceneInterface(synchronous=True)
        rospy.sleep(1.0)  # Wait for planning scene to initialize
        
        # Setup TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Store table positions
        self.table_positions = [(8.0, -1.0), (8.0, -2.0)]
        
        # Dictionary to track collision objects
        self.collision_objects = {'1': None, '2': None, '3': None, '4': None, 
                                '5': None, '6': None, '7': None, '8': None, '9': None}
        
        # First remove any existing collision objects
        self.scene.remove_world_object()
        rospy.sleep(0.5)
        
        # Create the static tables
        self.add_static_tables()
        
        # Subscribe to detections after setup is complete
        self.detection_sub = rospy.Subscriber('/detected_objects', Detections, self.process_detections)

    def add_static_tables(self):
        """Add static tables to the planning scene"""
        # First table
        table1_pose = PoseStamped()
        table1_pose.header.frame_id = "map"
        table1_pose.pose.position.x = self.table_positions[0][0]
        table1_pose.pose.position.y = self.table_positions[0][1]
        table1_pose.pose.position.z = 0
        table1_pose.pose.orientation.w = 1.0
        
        # Add first table
        self.scene.add_box("table1", table1_pose, size=(0.95, 0.95, 0.9))
        
        # Second table
        table2_pose = PoseStamped()
        table2_pose.header.frame_id = "map"
        table2_pose.pose.position.x = self.table_positions[1][0]
        table2_pose.pose.position.y = self.table_positions[1][1]
        table2_pose.pose.position.z = 0
        table2_pose.pose.orientation.w = 1.0
        
        # Add second table
        self.scene.add_box("table2", table2_pose, size=(0.95, 0.95, 0.9))
        
        rospy.sleep(0.5)  # Wait for the scene to update

    def process_detections(self, msg):
        """Process detected objects and add them to the planning scene"""
        try:
            # Get transform from camera frame to map frame
            transform = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0))
            
            for i, (pose, id_num) in enumerate(zip(msg.poses, msg.ids)):
                # Remove previous instance of this object if it exists
                obj_name = f"object_{id_num}"
                self.scene.remove_world_object(obj_name)
                
                # Convert detection pose to PoseStamped
                object_pose = PoseStamped()
                object_pose.header.frame_id = "base_link"
                object_pose.pose = pose
                
                # Transform pose to map frame
                map_pose = do_transform_pose(object_pose, transform)
                
                # Add object based on its type
                object_type = self.classify_object(id_num)
                self.add_collision_object(object_type, map_pose, obj_name)
                
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"TF Error: {e}")
        except Exception as e:
            rospy.logerr(f"Error processing detections: {e}")

    def add_collision_object(self, object_type, pose, name):
        """Add collision object to the planning scene"""
        try:
            if object_type == "cube":
                self.scene.add_box(name, pose, size=(0.06, 0.06, 0.06))
            elif object_type == "hexagonal prism":
                self.scene.add_cylinder(name, pose, height=0.3, radius=0.06)
            elif object_type == "triangular prism":
                self.scene.add_box(name, pose, size=(0.06, 0.04, 0.08))
        except Exception as e:
            rospy.logerr(f"Failed to add collision object {name}: {e}")

    def classify_object(self, tag_id):
        """Classify object based on AprilTag ID"""
        if tag_id in [1, 2, 3]:
            return "hexagonal prism"
        elif tag_id in [4, 5, 6]:
            return "cube"
        elif tag_id in [7, 8, 9]:
            return "triangular prism"
        else:
            return "unknown"

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = NodeC()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node C stopped.")

# import rospy
# from moveit_commander import MoveGroupCommander, PlanningSceneInterface
# from geometry_msgs.msg import PoseStamped
# from moveit_msgs.msg import CollisionObject
# from shape_msgs.msg import SolidPrimitive
# from ir2425_group_09.msg import Detections
# import tf2_ros
# from tf2_geometry_msgs import do_transform_pose

# class NodeC:
#     def __init__ (self):
#         rospy.init_node('nodeC')

#         # Define the subscriber to receive the detections
#         rospy.Subscriber('/detected_objects', Detections, self.process_detections)

#         self.table_positions = [(8.0, -1.0), (8.0, -2.0)]
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
#         self.collision_objects = {'1': None, '2': None, '3': None, '4': None, '5': None, '6': None, '7': None, '8': None, '9': None}
#         self.table_1 = CollisionObject()
#         self.table_2 = CollisionObject()
#         self.scene = PlanningSceneInterface()

#         self.create_table()

#     def create_table(self):
#         # Add tables to the scene
#         self.table_1.header.frame_id = "map"
#         self.table_1.id = "table1"
#         self.table_1.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.9, 0.9, 0.9]))
#         pose = PoseStamped()
#         pose.header.frame_id = "map"
#         pose.pose.position.x = self.table_positions[0][0]
#         pose.pose.position.y = self.table_positions[0][1]
#         pose.pose.position.z = 0
#         pose.pose.orientation.w = 1.0
#         self.table_1.primitive_poses.append(pose.pose)
#         self.table_1.operation = CollisionObject.ADD

#         print("Adding table 1 to the scene")
#         self.scene.add_object(self.table_1)

        
#         self.table_2.header.frame_id = "map"
#         self.table_2.id = "table2"
#         self.table_2.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.9, 0.9, 0.9]))
#         pose = PoseStamped()
#         pose.header.frame_id = "map"
#         pose.pose.position.x = self.table_positions[1][0]
#         pose.pose.position.y = self.table_positions[1][1]
#         pose.pose.position.z = 0
#         pose.pose.orientation.w = 1.0
#         self.table_2.primitive_poses.append(pose.pose)
#         self.table_2.operation = CollisionObject.ADD

#         print("Adding table 2 to the scene")
#         self.scene.add_object(self.table_2)

#     def process_detections(self, msg):
#         """
#         Process the detections received from node B
#         """
#         print("Upate the tables")
#         self.update_tables()

#         poses = msg.poses
#         ids = msg.ids
#         target = msg.target
#         print(f"target is : {target}")
#         print("received detections:")
#         for i in range(len(poses)):
#             id = ids[i]
#             pose = poses[i]
#             x = pose.position.x
#             y = pose.position.y
#             z = pose.position.z
#             w = pose.orientation.w
#             print(f"id: {id}, x = {x:.3f}, y = {y:.3f}, z = {z:.3f}, w = {w:.3f}")

#             # Convert pose to PoseStamped
#             pose_stamped = PoseStamped()
#             pose_stamped.header.frame_id = "base_link"
#             pose_stamped.pose = pose

#             # Classify and create collision object
#             object_type = self.classify_object(id)
#             object = self.create_collision_objects(object_type, pose_stamped)
#             self.collision_objects.update({str(id): object})
#             print(f"Created collision object for {object_type}")
#             self.scene.add_object(object)
#             print(f"Added collision object to the scene: {self.collision_objects[str(id)].id}")

#     def classify_object(self, tag_id):
#         """
#         Classify the objects based on  their tag id
#         """
#         if tag_id in [1, 2, 3]:
#             return "hexagonal prism"
#         elif tag_id in [4, 5, 6]:
#             return "cube"
#         elif tag_id in [7, 8, 9]:
#             return "triangular prism"
#         else:
#             return "table"

#     def create_collision_objects(self, object_type, pose):
#         object = CollisionObject()
#         object.header.frame_id = "base_link"
#         object.id = object_type
        
#         if object_type == "cube":
#             object.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.06, 0.06, 0.06]))
#             object.primitive_poses.append(pose.pose)
#         elif object_type == "hexagonal prism":
#             object.primitives.append(SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[0.3, 0.06]))
#             object.primitive_poses.append(pose.pose)
#         elif object_type == "triangular prism":
#             object.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.06, 0.04, 0.08]))
#             object.primitive_poses.append(pose.pose)
#         # elif object_type == "table":
#         #     object.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[1.0, 1.0, 1.0]))
#         #     pose_stamp = PoseStamped()
#         #     pose_stamp.header.frame_id = "base_link"

#         object.operation = CollisionObject.ADD
#         return object
    
#     def update_tables(self):
#         # Remove the old tables
#         self.scene.remove_world_object("table1")
#         self.scene.remove_world_object("table2")

#         # Update the pose of the first table
#         pose_old = self.table_1.header.frame_id
#         pose_new = PoseStamped()
#         pose_new.header.frame_id = "base_link"
#         pose_new.header.stamp = rospy.Time.now()

#         try:
#             transform = self.tf_buffer.lookup_transform("base_link", pose_old, rospy.Time(0))
#             pose_new = do_transform_pose(pose_new, transform)
#         except Exception as e:
#             rospy.logerr(f"Failed to transform pose for table: {e}")

#         self.table_1.primitive_poses[0] = pose_new.pose
#         self.scene.add_object(self.table_1)

#         # Update the pose of the second table
#         pose_old = self.table_2.header.frame_id
#         pose_new = PoseStamped()
#         pose_new.header.frame_id = "base_link"
#         pose_new.header.stamp = rospy.Time.now()

#         try:
#             transform = self.tf_buffer.lookup_transform("base_link", pose_old, rospy.Time(0))
#             pose_new = do_transform_pose(pose_new, transform)
#         except Exception as e:
#             rospy.logerr(f"Failed to transform pose for table: {e}")

#         self.table_2.primitive_poses[0] = pose_new.pose
#         self.scene.add_object(self.table_2)
        
#     def run(self):
#         rospy.spin()

# if __name__ == '__main__':
#     try:
#         node = NodeC()
#         node.run()
#     except rospy.ROSInterruptException:
#         rospy.loginfo("Node C stopped.")