import math
import rclpy
import tf_transformations
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class HappyMoveNode(Node):
    def __init__(self):
        super().__init__('happy_move_node')

        self.wheel_base = 0.287
        self.wheel_radius = 0.033

        self.vel = Twist()
        self.set_vel(0.0, 0.0)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 4)

        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.x0, self.y0, self.yaw0 = 0.0, 0.0, 0.0
        self.initialized = False

        self.sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 4)

        self.timer_hz = 100.0
        self.timer_count = 0

        self.timer = self.create_timer(
            1.0 / self.timer_hz, self.timer_callback)

    def get_pose(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(
            (q_x, q_y, q_z, q_w))
        return x, y, yaw

    def reset_origin(self):
        self.x0 = self.x
        self.y0 = self.y
        self.yaw0 = self.yaw

    def odom_callback(self, msg):
        self.x, self.y, self.yaw = self.get_pose(msg)
        left, right = self.calc_wheel_angular_velocity(msg)
        if self.initialized == False:
            self.reset_origin()
            self.initialized = True
        self.get_logger().info(
            f'(x,y,yaw){self.x: .3f} {self.y: .3f} {self.yaw: .3f} '
            f'(x0, y0, yaw0){self.x0: .3f} {self.y0: .3f} {self.yaw0: .3f} '
            f'(dist, yaw){math.sqrt((self.x - self.x0)**2 + (self.y - self.y0)**2): .3f} '
            f'{self.yaw - self.yaw0: .3f} '
            f'(left,right){left: .3f} {right: .3f}')

    def timer_callback(self):
        self.timer_count += 1
        self.pub.publish(self.vel)

    def set_vel(self, linear, angular):
        self.vel.linear.x = linear  # [m/s]
        self.vel.angular.z = angular  # [rad/s]

    def move_distance(self, dist):
        error = 0.02
        d0 = math.fabs(dist)
        d = math.sqrt((self.x - self.x0)**2 + (self.y - self.y0)**2)
        if d0 - d > error:
            self.set_vel(0.25, 0.0)
            return False
        else:
            self.set_vel(0.0, 0.0)
            return True

    def rotate_angle(self, angle):
        error = 0.05
        w = math.sin(angle - (self.yaw - self.yaw0))
        if w > error:
            self.set_vel(0.0, 0.25)
            return False
        else:
            self.set_vel(0.0, 0.0)
            return True

    def happy_move(self, distance, angle):
        state = 0
        while rclpy.ok():
            if self.initialized == False:
                pass
            elif state == 0:
                if self.move_distance(distance):
                    self.reset_origin()
                    state = 1
            elif state == 1:
                if self.rotate_angle(angle):
                    self.reset_origin()
                    break
            else:
                print('error state!!!')
            rclpy.spin_once(self)

    def move_time(self, time, linear_vel, angular_vel):
        end_count = self.timer_count + time * self.timer_hz
        while rclpy.ok():
            if self.initialized == False:
                pass
            elif self.timer_count < end_count:
                self.set_vel(linear_vel, angular_vel)
            else:
                break
            rclpy.spin_once(self)

    def draw_square(self, x):
        state = 0
        while rclpy.ok():
            if self.initialized == False:
                pass
            elif self.happy_move(x, math.pi / 2.0):
                self.reset_origin()
            rclpy.spin_once(self)

    def draw_circle(self, r):
        while rclpy.ok():
            if self.initialized == False:
                pass
            else:
                w = 0.25
                self.set_vel(r * w, w)
            rclpy.spin_once(self)

    @staticmethod
    def normailize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def drive_toward(self, x, y, v):
        while rclpy.ok():
            if self.initialized == False:
                pass
            else:
                dx = x - self.x
                dy = y - self.y
                yaw = math.atan2(dy, dx)
                yaw_error = self.normailize_angle(yaw - self.yaw)
                d = math.sqrt(dx * dx + dy * dy)
                if d < 0.02:
                    break
                else:
                    if abs(yaw_error) > 0.05:
                        self.set_vel(0.0, v if yaw_error > 0 else -v)
                    else:
                        self.set_vel(v, 0.0)
            rclpy.spin_once(self)

    def calc_wheel_angular_velocity(self, msg: Odometry):
        v = msg.twist.twist.linear.x   # 前進速度[m/s]
        w = msg.twist.twist.angular.z  # 回転速度[rad/s]

        left = (v - (w * self.wheel_base / 2.0)) / self.wheel_radius
        right = (v + (w * self.wheel_base / 2.0)) / self.wheel_radius

        return left, right


def main():
    rclpy.init()
    node = HappyMoveNode()
    try:
        # node.happy_move(2.0, math.pi / 2.0)
        # node.move_time(5.0, 0.25, 0.25)
        # node.draw_square(2.0)
        # node.draw_circle(1.0)
        node.drive_toward(2.0, 2.0, 0.25)
        node.set_vel(0.0, 0.0)
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        print('Pressed Ctrl+C')
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
