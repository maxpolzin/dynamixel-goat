import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

from .dynamixel_controller import Dynamixel


class GoatController(Node):
    def __init__(self):
        super().__init__('goat_controller')

        self.joystick_topic = self.declare_parameter('joystick_topic', '/joy').get_parameter_value().string_value
        self.commanded_velocity_topic = self.declare_parameter('commanded_velocity_topic', '/commanded_velocity').get_parameter_value().string_value
        self.measured_velocity_topic = self.declare_parameter('measured_velocity_topic', '/measured_velocity').get_parameter_value().string_value
        self.current_consumption_topic = self.declare_parameter('current_consumption_topic', '/current_consumption').get_parameter_value().string_value

        self.linear_scale = self.declare_parameter('linear_scale', 1.0).get_parameter_value().double_value
        self.angular_scale = self.declare_parameter('angular_scale', 1.0).get_parameter_value().double_value

        self.joystick_subscription = self.create_subscription(Joy, self.joystick_topic, self.joystick_callback, 10)
        self.commanded_velocity_publisher = self.create_publisher(Float32MultiArray, self.commanded_velocity_topic, 10)
        self.measured_velocity_publisher = self.create_publisher(Float32MultiArray, self.measured_velocity_topic, 10)
        self.current_consumption_publisher = self.create_publisher(Float32MultiArray, self.current_consumption_topic, 10)

        self.current_triangle_velocity = 0.0
        self.max_triangle_velocity = 1.0
        self.velocity_increment = 0.1

        self.previous_button_states = [0] * 13

        self.get_logger().info(f"Subscribed to {self.joystick_topic}")
        self.get_logger().info(f"Publishing commanded velocity to {self.commanded_velocity_topic}")
        self.get_logger().info(f"Publishing measured velocity to {self.measured_velocity_topic}")
        self.get_logger().info(f"Publishing current consumption to {self.current_consumption_topic}")

        self.servo = Dynamixel(ID=[0, 1, 2, 3], descriptive_device_name="DYNAMIXEL_GOAT", series_name=["xw", "xw", "xw", "xw"], baudrate=1000000, port_name="/dev/ttyUSB0")
        self.servo.begin_communication()
        self.servo.set_operating_mode("velocity", ID="all")

        self.ID_FRONT_LEFT = 2
        self.ID_BACK_LEFT = 0
        self.ID_FRONT_RIGHT = 3 
        self.ID_BACK_RIGHT = 1

        self.DIR_FRONT_LEFT = 1
        self.DIR_BACK_LEFT = 1
        self.DIR_FRONT_RIGHT = -1 
        self.DIR_BACK_RIGHT = -1


    def joystick_callback(self, msg: Joy):
        triangle_button = msg.buttons[2]
        x_button = msg.buttons[0]
        triangle_button_prev = self.previous_button_states[2]
        x_button_prev = self.previous_button_states[0]

        linear_velocity = self.current_triangle_velocity
        angular_velocity = 0.0

        if abs(msg.axes[1]) > 0.1 or abs(msg.axes[0]) > 0.1:
            linear_velocity = self.linear_scale * msg.axes[1]
            angular_velocity = self.angular_scale * msg.axes[0]
            self.current_triangle_velocity = 0.0
        elif triangle_button == 1 and triangle_button_prev == 0:
            self.current_triangle_velocity = min(
                self.current_triangle_velocity + self.velocity_increment, self.max_triangle_velocity
            )
            linear_velocity = self.current_triangle_velocity
            angular_velocity = 0.0
        elif x_button == 1 and x_button_prev == 0:
            self.current_triangle_velocity = 0.0
            linear_velocity = 0.0
            angular_velocity = 0.0

        left_wheel_velocity = linear_velocity - angular_velocity
        right_wheel_velocity = linear_velocity + angular_velocity

        left_wheel_dynamixel_velocity = int(left_wheel_velocity * 310)
        right_wheel_dynamixel_velocity = int(right_wheel_velocity * 310)

        self.servo.write_velocity(self.DIR_FRONT_LEFT * left_wheel_dynamixel_velocity, self.ID_FRONT_LEFT)
        self.servo.write_velocity(self.DIR_FRONT_RIGHT * right_wheel_dynamixel_velocity, self.ID_FRONT_RIGHT)
        self.servo.write_velocity(self.DIR_BACK_LEFT * left_wheel_dynamixel_velocity, self.ID_BACK_LEFT)
        self.servo.write_velocity(self.DIR_BACK_RIGHT * right_wheel_dynamixel_velocity, self.ID_BACK_RIGHT)

        left_wheel_velocity = left_wheel_dynamixel_velocity * 0.226
        right_wheel_velocity = right_wheel_dynamixel_velocity * 0.226

        # Publish commanded velocity
        commanded_velocity_msg = Float32MultiArray()
        commanded_velocity_msg.data = [left_wheel_velocity, right_wheel_velocity]
        self.commanded_velocity_publisher.publish(commanded_velocity_msg)

        # Publish measured velocity
        front_left_wheel_velocity_raw = self.DIR_FRONT_LEFT * self.servo.read_velocity(self.ID_FRONT_LEFT)
        back_left_wheel_velocity_raw = self.DIR_BACK_LEFT * self.servo.read_velocity(self.ID_BACK_LEFT)
        front_right_wheel_velocity_raw = self.DIR_FRONT_RIGHT * self.servo.read_velocity(self.ID_FRONT_RIGHT)
        back_right_wheel_velocity_raw = self.DIR_BACK_RIGHT * self.servo.read_velocity(self.ID_BACK_RIGHT)

        front_left_wheel_measured_velocity = front_left_wheel_velocity_raw * 0.226
        back_left_wheel_measured_velocity = back_left_wheel_velocity_raw * 0.226
        front_right_wheel_measured_velocity = front_right_wheel_velocity_raw * 0.226
        back_right_wheel_measured_velocity = back_right_wheel_velocity_raw * 0.226

        measured_velocity_msg = Float32MultiArray()
        measured_velocity_msg.data = [front_left_wheel_measured_velocity, back_left_wheel_measured_velocity, front_right_wheel_measured_velocity, back_right_wheel_measured_velocity]
        self.measured_velocity_publisher.publish(measured_velocity_msg)

        # Read and scale current consumption
        front_left_wheel_current_raw = self.servo.read_current(self.ID_FRONT_LEFT)
        back_left_wheel_current_raw = self.servo.read_current(self.ID_BACK_LEFT)
        front_right_wheel_current_raw = self.servo.read_current(self.ID_FRONT_RIGHT)
        back_right_wheel_current_raw = self.servo.read_current(self.ID_BACK_RIGHT)

        front_left_wheel_current = front_left_wheel_current_raw * 2.69e-3  # Scale raw current to Amps
        back_left_wheel_current =  back_left_wheel_current_raw * 2.69e-3  # Scale raw current to Amps
        front_right_wheel_current = self.DIR_FRONT_RIGHT * front_right_wheel_current_raw * 2.69e-3  # Scale raw current to Amps
        back_right_wheel_current = self.DIR_BACK_RIGHT * back_right_wheel_current_raw * 2.69e-3  # Scale raw current to Amps

        # Publish current consumption
        current_consumption_msg = Float32MultiArray()
        current_consumption_msg.data = [front_left_wheel_current, back_left_wheel_current, front_right_wheel_current, back_right_wheel_current]
        self.current_consumption_publisher.publish(current_consumption_msg)

        self.previous_button_states = msg.buttons


def main(args=None):
    rclpy.init(args=args)
    goat_controller_node = GoatController()
    rclpy.spin(goat_controller_node)
    goat_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
