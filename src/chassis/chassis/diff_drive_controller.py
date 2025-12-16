import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class DiffDriveController(Node):
    def __init__(self):
        super().__init__('diff_drive_controller')
        # parameters (match URDF values)
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('wheel_separation', 0.18288)
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value

        # state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        self.v = 0.0
        self.omega = 0.0

        self.sub = self.create_subscription(Twist, '/diff_drive/cmd_vel', self.cmd_cb, 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/diff_drive/odometry', 10)
        self.br = TransformBroadcaster(self)

        self.timer = self.create_timer(1.0/50.0, self.update)
        self.last_time = self.get_clock().now()

    def cmd_cb(self, msg: Twist):
        self.v = msg.linear.x
        self.omega = msg.angular.z

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_time = now

        # integrate pose
        dx = self.v * math.cos(self.yaw) * dt
        dy = self.v * math.sin(self.yaw) * dt
        self.x += dx
        self.y += dy
        self.yaw += self.omega * dt

        # wheel angles
        v_r = self.v + (self.omega * self.wheel_separation / 2.0)
        v_l = self.v - (self.omega * self.wheel_separation / 2.0)
        ang_r = v_r / self.wheel_radius
        ang_l = v_l / self.wheel_radius
        self.right_wheel_angle += ang_r * dt
        self.left_wheel_angle += ang_l * dt

        # publish joint states
        js = JointState()
        js.header = Header()
        js.header.stamp = now.to_msg()
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [self.left_wheel_angle, self.right_wheel_angle]
        self.joint_pub.publish(js)

        # publish odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        # compute quaternion from yaw (no roll/pitch)
        qx = 0.0
        qy = 0.0
        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.omega
        self.odom_pub.publish(odom)

        # publish tf odom -> base_link
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
