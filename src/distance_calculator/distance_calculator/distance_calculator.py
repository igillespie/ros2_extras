import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import math

class DistanceCalculator(Node):
    def __init__(self):
        super().__init__('distance_calculator')

        # Subscriber to /odom
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10  # QoS depth
        )

        # Publisher to /distance_traveled
        self.distance_publisher = self.create_publisher(
            Float32,
            '/distance_traveled',
            10  # QoS depth
        )

        # Variables to store previous position and total distance
        self.prev_x = None
        self.prev_y = None
        self.total_distance = 0.0

        # Noise threshold (in meters)
        self.noise_threshold = 0.01  # 1 cm

        # self.get_logger().info("DistanceCalculator node has been started.")

    def odom_callback(self, msg):

        # Extract the current position from the Odometry message
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        #self.get_logger().info(f"Current: ({current_x}, {current_y}), Previous: ({self.prev_x}, {self.prev_y})")

        # Calculate distance only if we have a previous position
        if self.prev_x is not None and self.prev_y is not None:
            dx = current_x - self.prev_x
            dy = current_y - self.prev_y

            # Ignore small movements due to noise
            if abs(dx) < self.noise_threshold and abs(dy) < self.noise_threshold:
                return

            # Calculate the Euclidean distance
            distance = math.sqrt(dx ** 2 + dy ** 2)

            # Update total distance
            self.total_distance += distance

            # Publish total distance
            distance_msg = Float32()
            distance_msg.data = self.total_distance
            self.distance_publisher.publish(distance_msg)

            #self.get_logger().info(f"Distance traveled: {self.total_distance:.2f} meters")

        # Update previous position
        self.prev_x = current_x
        self.prev_y = current_y


def main(args=None):
    rclpy.init(args=args)
    node = DistanceCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()