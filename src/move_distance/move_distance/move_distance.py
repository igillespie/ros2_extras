import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class MoveDistanceNode(Node):
    def __init__(self):
        super().__init__('move_distance')
        self.declare_parameter('linear_speed', 0.2)  # Default linear speed in m/s

        # Subscribers
        self.create_subscription(Float32, '/move_distance', self.move_callback, 10)
        self.create_subscription(Float32, '/distance_traveled', self.distance_callback, 10)

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Internal state
        self.target_distance = None
        self.initial_distance = None
        self.current_distance = 0.0
        self.linear_speed = self.get_parameter('linear_speed').value
        self.moving = False  # Flag to indicate if the robot is actively moving
        self.velocity_timer = None  # Timer will be created dynamically

        self.get_logger().info('MoveDistanceNode is ready.')

    def move_callback(self, msg):
        """
        Callback for /move_distance. Starts the robot moving toward the target distance.
        """
        self.target_distance = msg.data
        self.initial_distance = None  # Reset initial distance
        self.moving = True
        self.get_logger().info(f'Received target distance: {self.target_distance} meters')

        # Start the timer for publishing velocity
        self.start_velocity_timer()

    def distance_callback(self, msg):
        """
        Callback for /distance_traveled. Updates the current distance and checks if the target is reached.
        """
        self.current_distance = msg.data

        # Record the initial distance when movement starts
        if self.initial_distance is None:
            self.initial_distance = self.current_distance
            self.get_logger().info(f'Initial distance recorded: {self.initial_distance} meters')

        # Calculate distance traveled relative to the initial distance
        distance_traveled = abs(self.current_distance - self.initial_distance)

        # Stop the robot if the target distance is reached or exceeded
        if self.moving and distance_traveled >= abs(self.target_distance):
            self.stop_robot()

    def start_velocity_timer(self):
        """
        Starts the timer for continuous velocity publishing.
        """
        if self.velocity_timer is None:
            self.velocity_timer = self.create_timer(0.1, self.publish_velocity)  # Publish every 0.1 seconds
            self.get_logger().debug("Velocity timer started.")

    def stop_velocity_timer(self):
        """
        Stops the velocity publishing timer.
        """
        if self.velocity_timer is not None:
            self.velocity_timer.cancel()
            self.velocity_timer = None
            self.get_logger().debug("Velocity timer stopped.")

    def publish_velocity(self):
        """
        Publishes velocity commands to /cmd_vel if the robot is moving.
        """
        if self.moving:
            twist = Twist()
            twist.linear.x = self.linear_speed if self.target_distance > 0 else -self.linear_speed
            self.cmd_vel_pub.publish(twist)
            self.get_logger().debug(f'Publishing velocity: {twist.linear.x} m/s')

    def stop_robot(self):
        """
        Stops the robot by publishing zero velocity, stopping the timer, and resetting the state.
        """
        twist = Twist()
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        self.moving = False
        self.target_distance = None  # Reset target distance
        self.stop_velocity_timer()  # Stop the velocity timer
        self.get_logger().debug('Robot stopped. Movement completed.')


def main(args=None):
    rclpy.init(args=args)
    node = MoveDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()