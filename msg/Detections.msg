std_msgs/Header header
geometry_msgs/Pose[] poses           # Array of object poses
int32[] ids                          # Array of corresponding tag IDs
string[] types                      # Array of types of the object
string task                         # what phase ("picking" or "placing")