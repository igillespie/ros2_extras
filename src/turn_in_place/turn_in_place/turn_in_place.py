import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turn_in_place_interfaces.msg import TurnCommand
from math import radians, copysign
from time import sleep

class TurnInPlaceNode(Node):
    def __init__(self):
        super().__init__('turn_in_place')
        self.publisher = self.create_publisher(Twist, 'cmd_vel_intuitive', 10)
        self.subscriber = self.create_subscription(
            TurnCommand,
            'turn_command',
            self.execute_turn,
            10
        )
        self.get_logger().info('TurnInPlaceNode is ready and listening for turn commands.')

    def execute_turn(self, msg):
        """
        Execute a turn based on the received TurnCommand message.
        """
        angle_degrees = msg.angle_degrees
        angular_speed_deg_per_sec = msg.angular_speed_deg_per_sec
        self.get_logger().info(f"Received command to turn {angle_degrees}˚ at {angular_speed_deg_per_sec}˚/s.")
        self.turn(angle_degrees, angular_speed_deg_per_sec)

    def turn(self, angle_degrees: float, angular_speed_deg_per_sec: float = 30.0):
        twist = Twist()
        angular_speed = radians(angular_speed_deg_per_sec)
        angle_radians = radians(angle_degrees)

        twist.angular.y = copysign(angular_speed, angle_radians)
        duration = abs(angle_radians) / angular_speed

        self.get_logger().info(f"Starting turn: {angle_degrees}˚ at {angular_speed_deg_per_sec}˚/s "
                           f"(angular_speed: {angular_speed:.2f} rad/s, "
                           f"angle: {angle_radians:.2f} rad, duration: {duration:.2f}s)")

        end_time = self.get_clock().now().to_msg().sec + (duration * 1.5)
        while self.get_clock().now().to_msg().sec < end_time:
            self.publisher.publish(twist)
            sleep(0.1)

        twist.angular.z = 0.0
        twist.angular.y = 0.0
        self.publisher.publish(twist)
        self.get_logger().info("Turn complete. Robot stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = TurnInPlaceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()