# Lock state constants
uint8 IDLE=0
uint8 LOCKED=1
uint8 LOST=2

std_msgs/Header header
int32 lock_id
uint8 lock_state
TrackedPerson[] persons
