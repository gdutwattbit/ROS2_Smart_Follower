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
# L2-normalized appearance feature used by ReID/tracking.
float32[512] appearance_feature
builtin_interfaces/Time last_seen
