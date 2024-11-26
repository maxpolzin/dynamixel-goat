import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class GoatController(Node):
    def __init__(self):
        super().__init__('goat_controller')

        # Declare parameters for joystick topic and scale factors
        self.joystick_topic = self.declare_parameter('joystick_topic', '/joystick').get_parameter_value().string_value
        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').get_parameter_value().string_value
        self.linear_scale = self.declare_parameter('linear_scale', 1.0).get_parameter_value().double_value
        self.angular_scale = self.declare_parameter('angular_scale', 1.0).get_parameter_value().double_value

        # Create a subscriber for joystick input
        self.joystick_subscription = self.create_subscription(Joy, self.joystick_topic, self.joystick_callback, 10)

        # Create a publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.get_logger().info(f"Subscribed to {self.joystick_topic}")
        self.get_logger().info(f"Publishing velocity commands to {self.cmd_vel_topic}")

    def joystick_callback(self, msg: Joy):

        linear_velocity = self.linear_scale * msg.axes[1]  # Assuming forward/backward on the left stick (axis 1)
        angular_velocity = self.angular_scale * msg.axes[0]  # Assuming left/right on the left stick (axis 0)

        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    goat_controller_node = GoatController()

    rclpy.spin(goat_controller_node)

    # Cleanup
    goat_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
