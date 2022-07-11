import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import quaternion_multiply, quaternion_inverse


class DummySensorSubscriber(Node):

    def __init__(self):
        super().__init__('dummy_sensor_subscriber')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscription = self.create_subscription(
            PointStamped, 'point', self.subscriber_callback, 10)
        self.publisher = self.create_publisher(PointStamped, 'point2', 10)

    def subscriber_callback(self, msg):
        try:
            when = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'base_link', 'dummy_sensor', when)
        except TransformException as ex:
            self.get_logger().info(str(ex))
            return
        mp = msg.point
        t = trans.transform.translation
        rq = trans.transform.rotation
        q = (rq.x, rq.y, rq.z, rq.w)
        p = (mp.x, mp.y, mp.z, 0)
        p2 = quaternion_multiply(
            quaternion_multiply(q, p),
            quaternion_inverse(q))
        msg2 = PointStamped()
        msg2.header.stamp = msg.header.stamp
        msg2.header.frame_id = 'base_link'
        msg2.point.x = p2[0] + t.x
        msg2.point.y = p2[1] + t.y
        msg2.point.z = p2[2] + t.z
        self.publisher.publish(msg2)


def main():
    rclpy.init()
    node = DummySensorSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
