import rclpy
from rclpy.node import Node
from mast_control_interfaces.msg import PanTiltCommand, PanTiltState
from adafruit_servokit import ServoKit
from pca9685_kit.pca9685_kit import PCA9685Kit


class MastControl(Node):
    def __init__(self):
        super().__init__('mast_control')

        # Initialize PCA9685Kit
        self.servo_kit = ServoKit(channels=16)
        self.pca9685 = PCA9685Kit(self.servo_kit)

        # Declare parameters for servo channels
        self.declare_parameter('pan_channel', 4)
        self.declare_parameter('tilt_channel', 5)
        self.declare_parameter('centered_pan_angle', 150)
        self.declare_parameter('centered_tilt_angle', 150)

        # Retrieve parameters
        self.pan_channel = self.get_parameter('pan_channel').value
        self.tilt_channel = self.get_parameter('tilt_channel').value
        self.centered_pan_angle = self.get_parameter('centered_pan_angle').value
        self.centered_tilt_angle = self.get_parameter('centered_tilt_angle').value

        self.current_pan = 0.0
        self.current_tilt = 0.0

        # Configure the servos
        self.pca9685.configure_servo(self.pan_channel, actuation_range=300, min_pulse=500, max_pulse=2500)
        self.pca9685.configure_servo(self.tilt_channel, actuation_range=300, min_pulse=500, max_pulse=2500)

        # State publisher
        self.state_pub = self.create_publisher(PanTiltState, '/pan_tilt_state', 10)

        self.enc_pub_timer_period = 0.1  # [s]
        self.enc_pub_timer = self.create_timer(self.enc_pub_timer_period, self.publish_encoder_estimate)

        # Command subscriber
        self.command_sub = self.create_subscription(
            PanTiltCommand,
            '/pan_tilt_command',
            self.handle_command,
            10
        )

        self.get_logger().info("MastControl node has been started. Centering servos.")
        
        # Move servos to centered positions on startup
        self.move_mast_to_angles(self.centered_pan_angle, self.centered_tilt_angle)
        
    
    def move_mast_to_angles(self, pan_angle, tilt_angle):
        """
        Moves the servos to the specified pan and tilt angles.
        """
        try:
            # Validate and move servos
            self.pca9685.move_to_angle(self.pan_channel, pan_angle)
            self.pca9685.move_to_angle(self.tilt_channel, tilt_angle)

            # Update current state
            self.current_pan = pan_angle
            self.current_tilt = tilt_angle

            self.get_logger().debug(f"Moved mast to angles: pan={pan_angle}, tilt={tilt_angle}")
        except ValueError as e:
            self.get_logger().error(f"Error moving servos: {e}")

    def handle_command(self, msg):
        """
        Handles incoming PanTiltCommand messages and moves the mast accordingly.
        """
        self.get_logger().debug(f"Received command: pan={msg.pan_angle}, tilt={msg.tilt_angle}")
        self.move_mast_to_angles(msg.pan_angle, msg.tilt_angle)
       

    def publish_state(self, status):
        """
        Publishes the current state of the mast.
        """
        state_msg = PanTiltState()
        state_msg.pan_position = float(self.current_pan) if self.current_pan is not None else 0.0
        state_msg.tilt_position = float(self.current_tilt) if self.current_tilt is not None else 0.0
        state_msg.status = status
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.header.frame_id = "mast_frame"
        self.state_pub.publish(state_msg)

    def publish_encoder_estimate(self):
        self.publish_state("OK")

def main(args=None):
    rclpy.init(args=args)
    node = MastControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()