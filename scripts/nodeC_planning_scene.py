import rospy
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject 
from shape_msgs.msg import SolidPrimitive
from ir2425_group_09.msg import Detections
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from std_msgs.msg import String

class nodeC_planning_scene:
    def __init__ (self):
        rospy.init_node('nodeC_planning_scene')

        self.initial_time = rospy.Time.now()
        # Subscribe to the detected_objects topic
        rospy.Subscriber('/detected_objects', Detections, self.detection_callback)
        rospy.Subscriber('table_co', String, self.table_callback)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.table_positions = [(7.8, -2.0), (7.8, -3.0)]  # in order placing table, picking table
        
        # Initialize the PlanningSceneInterface
        self.scene = PlanningSceneInterface() # you don't need to publish it, handled automatically
        
        # definition of real object dimensions
        scale_factor_table = 1.08
        scale_factor_obj = 1.05
        self.objects_dimensions = {
            "pickup_table" : [0.9 * scale_factor_table, 0.9 * scale_factor_table, 0.78 * scale_factor_table],
            "placement_table" : [0.9 * scale_factor_table, 0.95 * scale_factor_table, 0.78 * scale_factor_table],
            "cube" : [0.05 * scale_factor_obj, 0.05 * scale_factor_obj, 0.05 * scale_factor_obj],
            "hexagonal prism" : [0.1 * scale_factor_obj, 0.025 * scale_factor_obj],
            "triangular prism" : [0.07, 0.05, 0.035]
        }

        self.current_objects = []

        self.picking_table_pose = self.create_table_pose("picking") # static table pose (3D center)
        self.placing_table_pose = self.create_table_pose("placing") # static table pose (3D center)

    def create_table_pose(self, table_type):
        """
        pose of the geometric center of the table (3D solid object) in map frame
        """
        table_pose = PoseStamped()
        table_pose.header.frame_id = "map"

        index = 1 if table_type == "picking" else 0         # access the correct table position
        table_pose.pose.position.x = self.table_positions[index][0]
        table_pose.pose.position.y = self.table_positions[index][1]

        table_pose.pose.position.z = self.objects_dimensions["pickup_table"][2]/2 
        table_pose.pose.orientation.w = 1.0

        return table_pose
    
    def table_callback(self, msg):
        """
        Callback for the table position
        """
        table_type = msg.data
        self.update_table(table_type)

    def detection_callback(self, msg):
        """
        Process the objects detected by nodeB, creating a suitable collision object for each detection, and for the table.
        The target pose is then published to nodeC_manipulator, in order to grab it
        """
        poses = msg.poses
        ids = msg.ids
        types = msg.types
        task = msg.task
        count_obj = 0
        
        for i in range(len(poses)):
            id = ids[i]
            pose = poses[i]
            type = types[i]
          
            collision_obj = self.get_collision_object(id, type, pose)
            if collision_obj:
                # Add the collision object to the planning scene
                self.scene.add_object(collision_obj)
                self.current_objects.append(collision_obj)
                count_obj += 1

        # Add the pickup table to the planning scene interface
        self.update_table("picking")

        # Add the placing table to the planning scene interface
        self.update_table("placing")

        rospy.loginfo(f"Created collsion objects (table + {count_obj} objects)")

    
    def update_table(self, table_type):
        """
        Updates the collision object for the table in the MoveIt planning scene.
        """

        object_name = "pickup_table" if table_type == "picking" else "placement_table"
        table_pose = self.picking_table_pose if table_type == "picking" else self.placing_table_pose

        try:
            # Define the table collision object
            table_collision_object = CollisionObject()
            table_collision_object.id = object_name
            table_collision_object.header.frame_id = "base_link"
            table_collision_object.operation = CollisionObject.ADD

            # Define the table's shape and dimensions
            table_shape = SolidPrimitive()
            table_shape.type = SolidPrimitive.BOX
            table_shape.dimensions = self.objects_dimensions[object_name]  # [x, y, z] dimensions of the table
            table_collision_object.primitives.append(table_shape)

            # Get and transform the table pose
            transform = self.tfBuffer.lookup_transform("base_link", table_pose.header.frame_id, rospy.Time(0))
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = table_pose.header.frame_id
            pose_stamped.pose = table_pose.pose

            # Transform the pose to base_link frame
            transformed_pose = do_transform_pose(pose_stamped, transform)

            # Adjust pose to ensure it aligns with the center of the table

            table_collision_object.primitive_poses.append(transformed_pose.pose)

            # Remove the existing table and add the updated one
            self.scene.remove_world_object(object_name)
            self.scene.add_object(table_collision_object)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Transform error while updating table: {e}")
        except Exception as e:
            rospy.logerr(f"Failed to update table collision object: {e}")


    def get_collision_object(self, id, type, pose):
        """
        produce a collision object from the object pose
        """
        obj = CollisionObject()
        obj.id = str(id)
        obj.header.frame_id = "base_link"
        obj.header.stamp = rospy.Time.now()

        if type == "cube":
            obj.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=self.objects_dimensions["cube"]))
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.pose = pose
            pose_stamped.pose.position.z -= self.objects_dimensions["cube"][2] / 2  # center the pose
            obj.primitive_poses.append(pose_stamped.pose)
            obj.operation = CollisionObject.ADD
            return obj

        elif type == "hexagonal prism":
            obj.primitives.append(SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=self.objects_dimensions["hexagonal prism"]))
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.pose = pose 
            pose_stamped.pose.position.z -= self.objects_dimensions["hexagonal prism"][0] / 2  # center the pose
            obj.primitive_poses.append(pose_stamped.pose)
            obj.operation = CollisionObject.ADD
            return obj

        elif type == "triangular prism":
            obj.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=self.objects_dimensions["triangular prism"]))
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.pose = pose
            obj.primitive_poses.append(pose_stamped.pose)
            obj.operation = CollisionObject.ADD
            return obj

        return None
    
if __name__ == '__main__':
    node = nodeC_planning_scene()
    rospy.spin()     
