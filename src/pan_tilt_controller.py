import rclpy
from rclpy.node import Node
from mast_control_interfaces.msg import PanTiltCommand, PanTiltState

class PanTiltController(Node):
    def __init__(self):
        super().__init__('pan_tilt_controller')

        # Initialize parameters for GPIO pins or motor control
        self.declare_parameter('pan_pin', 17)
        self.declare_parameter('tilt_pin', 27)

        self.pan_pin = self.get_parameter('pan_pin').value
        self.tilt_pin = self.get_parameter('tilt_pin').value

        # Initialize current positions
        self.current_pan = 0.0
        self.current_tilt = 0.0

        # State publisher
        self.state_pub = self.create_publisher(PanTiltState, '/pan_tilt_state', 10)

        # Command subscriber
        self.command_sub = self.create_subscription(
            PanTiltCommand,
            '/pan_tilt_command',
            self.handle_command,
            10
        )

        self.get_logger().info("Pan-Tilt Controller Node started.")

    def handle_command(self, msg):
        # Log received command
        self.get_logger().info(f"Received command: pan={msg.pan_angle}, tilt={msg.tilt_angle}, speed={msg.speed}")

        # Simulate moving the pan/tilt motors
        self.current_pan = msg.pan_angle
        self.current_tilt = msg.tilt_angle

        # Publish state feedback
        self.publish_state("OK")

    def publish_state(self, status):
        # Publish the current state of the pan-tilt mechanism
        state_msg = PanTiltState()
        state_msg.pan_position = self.current_pan
        state_msg.tilt_position = self.current_tilt
        state_msg.status = status
        self.state_pub.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PanTiltController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
