import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class UR5HighLevelController(Node):
    def __init__(self):
        super().__init__("ur5_high_level_controller")

        self._latest_joy: Joy = None

        self._declare_parameters()
        self._setup_publishers_subscribers()

        publish_period = 1.0 / self.publish_rate
        self.control_timer = self.create_timer(publish_period, self._control_callback)

    def _declare_parameters(self):
        self.joystick_topic = (
            self.declare_parameter("joystick_topic", "/joy").get_parameter_value().string_value
        )
        self.desired_velocity_topic = (
            self.declare_parameter("desired_velocity_topic", "/desired_velocity")
            .get_parameter_value()
            .string_value
        )

        self.max_linear_velocity = (
            self.declare_parameter("max_linear_velocity", 0.1).get_parameter_value().double_value
        )
        self.max_angular_velocity = (
            self.declare_parameter("max_angular_velocity", 0.1).get_parameter_value().double_value
        )
        self.deadzone = (
            self.declare_parameter("deadzone", 0.05).get_parameter_value().double_value
        )
        self.publish_rate = (
            self.declare_parameter("publish_rate", 50.0).get_parameter_value().double_value
        )

        self.axis_linear_x = (
            self.declare_parameter("axis_linear_x", 0).get_parameter_value().integer_value
        )
        self.axis_linear_y = (
            self.declare_parameter("axis_linear_y", 1).get_parameter_value().integer_value
        )
        self.axis_angular_z = (
            self.declare_parameter("axis_angular_z", 3).get_parameter_value().integer_value
        )

        self.button_r1 = (
            self.declare_parameter("button_r1", 5).get_parameter_value().integer_value
        )
        self.button_r2 = (
            self.declare_parameter("button_r2", 7).get_parameter_value().integer_value
        )
        self.button_dpad_left = (
            self.declare_parameter("button_dpad_left", 15).get_parameter_value().integer_value
        )
        self.button_dpad_right = (
            self.declare_parameter("button_dpad_right", 16).get_parameter_value().integer_value
        )
        self.button_dpad_up = (
            self.declare_parameter("button_dpad_up", 13).get_parameter_value().integer_value
        )
        self.button_dpad_down = (
            self.declare_parameter("button_dpad_down", 14).get_parameter_value().integer_value
        )

        self.invert_linear_y = (
            self.declare_parameter("invert_linear_y", False).get_parameter_value().bool_value
        )
        self.invert_angular_z = (
            self.declare_parameter("invert_angular_z", False).get_parameter_value().bool_value
        )

    def _setup_publishers_subscribers(self):
        self.joystick_subscription = self.create_subscription(
            Joy, self.joystick_topic, self._joystick_callback, 10
        )
        self.desired_velocity_publisher = self.create_publisher(
            Twist, self.desired_velocity_topic, 10
        )

    def _joystick_callback(self, msg: Joy):
        self._latest_joy = msg

    @staticmethod
    def _apply_deadzone(value: float, deadzone: float) -> float:
        if abs(value) < deadzone:
            return 0.0
        return value

    def _get_axis(self, msg: Joy, index: int, invert: bool = False) -> float:
        if index < 0 or index >= len(msg.axes):
            return 0.0
        value = msg.axes[index]
        if invert:
            value = -value
        return self._apply_deadzone(value, self.deadzone)

    def _get_button(self, msg: Joy, index: int) -> bool:
        if index < 0 or index >= len(msg.buttons):
            return False
        return msg.buttons[index] == 1

    def _control_callback(self):
        twist = Twist()

        if self._latest_joy is not None:
            msg = self._latest_joy

            vx = self._get_axis(msg, self.axis_linear_x) * self.max_linear_velocity
            vy = self._get_axis(msg, self.axis_linear_y, self.invert_linear_y) * self.max_linear_velocity

            if self._get_button(msg, self.button_r2):
                vz = self.max_linear_velocity
            elif self._get_button(msg, self.button_r1):
                vz = -self.max_linear_velocity
            else:
                vz = 0.0

            roll = 0.0
            if self._get_button(msg, self.button_dpad_right):
                roll = self.max_angular_velocity
            elif self._get_button(msg, self.button_dpad_left):
                roll = -self.max_angular_velocity

            pitch = 0.0
            if self._get_button(msg, self.button_dpad_up):
                pitch = self.max_angular_velocity
            elif self._get_button(msg, self.button_dpad_down):
                pitch = -self.max_angular_velocity

            yaw = self._get_axis(msg, self.axis_angular_z, self.invert_angular_z) * self.max_angular_velocity

            twist.linear.x = float(np.clip(vx, -self.max_linear_velocity, self.max_linear_velocity))
            twist.linear.y = float(np.clip(vy, -self.max_linear_velocity, self.max_linear_velocity))
            twist.linear.z = float(np.clip(vz, -self.max_linear_velocity, self.max_linear_velocity))
            twist.angular.x = float(np.clip(roll, -self.max_angular_velocity, self.max_angular_velocity))
            twist.angular.y = float(np.clip(pitch, -self.max_angular_velocity, self.max_angular_velocity))
            twist.angular.z = float(np.clip(yaw, -self.max_angular_velocity, self.max_angular_velocity))

        self.desired_velocity_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    controller = UR5HighLevelController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        if controller is not None:
            controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
