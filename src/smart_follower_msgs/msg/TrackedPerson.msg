# Track state constants
uint8 TENTATIVE=0
uint8 CONFIRMED=1
uint8 LOST=2

int32 track_id
uint8 track_state
float32 confidence
# Bounding box in synchronized color image pixel coordinates.
sensor_msgs/RegionOfInterest bbox
# Estimated position in PersonPoseArray.header.frame_id.
geometry_msgs/Point position
# 3D velocity in PersonPoseArray.header.frame_id.
geometry_msgs/Vector3 velocity
# Estimated range in meters; NaN when no direct depth measurement exists.
float32 depth_m
float32[2048] appearance_feature
builtin_interfaces/Time last_seen
