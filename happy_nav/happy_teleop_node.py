import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class HappyTeleopNode(Node):
    def __init__(self):
        super().__init__('happy_teleop_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 4)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.vel = Twist()
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0

    def timer_callback(self):
        key = input('input f, b, r, l, or s, then Enter << ')
        if key == 'f':
            self.vel.linear.x += 0.1
        elif key == 'b':
            self.vel.linear.x -= 0.1
        elif key == 'l':
            self.vel.angular.z += 0.1
        elif key == 'r':
            self.vel.angular.z -= 0.1
        elif key == 's':
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
        else:
            print('Invalid input')
        self.pub.publish(self.vel)
        self.get_logger().info(
            f'linear={self.vel.linear.x}, angular={self.vel.angular.z}')


def main():
    rclpy.init()
    node = HappyTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
