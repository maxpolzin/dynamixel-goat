import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


from .dynamixel_controller import Dynamixel



class GoatController(Node):
    def __init__(self):
        super().__init__('goat_controller')

        self.joystick_topic = self.declare_parameter('joystick_topic', '/joystick').get_parameter_value().string_value
        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').get_parameter_value().string_value
        self.linear_scale = self.declare_parameter('linear_scale', 1.0).get_parameter_value().double_value
        self.angular_scale = self.declare_parameter('angular_scale', 1.0).get_parameter_value().double_value

        self.joystick_subscription = self.create_subscription(Joy, self.joystick_topic, self.joystick_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.current_velocity_scale = 1.0
        self.max_velocity_scale = 1.0
        self.min_velocity_scale = 0.0
        self.velocity_increment = 0.1

        self.get_logger().info(f"Subscribed to {self.joystick_topic}")
        self.get_logger().info(f"Publishing velocity commands to {self.cmd_vel_topic}")

        servo = Dynamixel(ID=[1,2, 3, 4], descriptive_device_name="BAZINGA", series_name=["xm", "xm"], baudrate=1000000, port_name="/dev/ttyUSB0")

        servo.begin_communication()
        servo.set_operating_mode("velocity", ID = "all")
        
        servo.read_current(ID = "all")

    def joystick_callback(self, msg: Joy):
        x_button = msg.buttons[0]
        triangle_button = msg.buttons[3]

        if x_button == 1:
            self.current_velocity_scale = 0.0

        if triangle_button == 1:
            if self.current_velocity_scale < self.max_velocity_scale:
                self.current_velocity_scale = min(
                    self.current_velocity_scale + self.velocity_increment, self.max_velocity_scale
                )

        linear_velocity = self.linear_scale * msg.axes[1] * self.current_velocity_scale
        angular_velocity = self.angular_scale * msg.axes[0] * self.current_velocity_scale

        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    goat_controller_node = GoatController()
    rclpy.spin(goat_controller_node)
    goat_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
