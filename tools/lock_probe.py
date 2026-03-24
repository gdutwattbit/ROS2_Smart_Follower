import rclpy
from rclpy.node import Node
from smart_follower_msgs.msg import PersonPoseArray, FollowCommand

class Probe(Node):
    def __init__(self):
        super().__init__('lock_probe')
        self.count = 0
        self.latest = None
        self.samples = []
        self.sent = False
        self.sub = self.create_subscription(PersonPoseArray, '/robot1/person_pose', self.cb, 10)
        self.pub = self.create_publisher(FollowCommand, '/robot1/follow_command', 10)
        self.timer = self.create_timer(2.0, self.send_lock_once)

    def send_lock_once(self):
        if self.sent:
            return
        msg = FollowCommand()
        msg.command = FollowCommand.LOCK
        msg.target_id = -1
        self.pub.publish(msg)
        self.sent = True
        self.get_logger().info('sent LOCK')

    def cb(self, msg):
        self.count += 1
        persons = []
        for p in msg.persons[:3]:
            persons.append({
                'id': int(p.track_id),
                'state': int(p.track_state),
                'conf': round(float(p.confidence), 3),
                'bbox': [int(p.bbox.x_offset), int(p.bbox.y_offset), int(p.bbox.width), int(p.bbox.height)],
                'pos': [round(float(p.position.x), 3), round(float(p.position.y), 3)],
            })
        self.latest = {
            'lock_id': int(msg.lock_id),
            'lock_state': int(msg.lock_state),
            'persons': persons,
        }
        if len(self.samples) < 12:
            self.samples.append(self.latest)

rclpy.init()
node = Probe()
end_ns = node.get_clock().now().nanoseconds + int(12e9)
while rclpy.ok() and node.get_clock().now().nanoseconds < end_ns:
    rclpy.spin_once(node, timeout_sec=0.2)
print('count=', node.count)
print('samples=')
for i, s in enumerate(node.samples):
    print(i, s)
print('latest=', node.latest)
node.destroy_node()
rclpy.shutdown()
