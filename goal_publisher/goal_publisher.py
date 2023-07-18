import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose


class GoalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Pose, 'robot_arm_goal', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Pose()
        msg.position.x = 0.
        msg.position.y = 0.
        msg.position.z = 0.
        msg.orientation.x = 0.
        msg.orientation.y = 0.
        msg.orientation.z = 0.
        msg.orientation.w = 1.
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = GoalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()