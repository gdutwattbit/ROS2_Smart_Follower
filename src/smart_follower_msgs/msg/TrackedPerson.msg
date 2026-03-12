# Track state constants
uint8 TENTATIVE=0
uint8 CONFIRMED=1
uint8 LOST=2

int32 track_id
uint8 track_state
float32 confidence
sensor_msgs/RegionOfInterest bbox
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float32 depth_m
float32[128] appearance_feature
builtin_interfaces/Time last_seen
