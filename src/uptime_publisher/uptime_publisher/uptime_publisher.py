from std_msgs.msg import Float32
import rclpy
from rclpy.node import Node

class UptimePublisher(Node):
    def __init__(self):
        super().__init__('uptime_publisher')
        self.start_time = self.get_clock().now()
        
        # Create a publisher for uptime
        self.publisher = self.create_publisher(Float32, 'uptime', 10)
        
        # Timer to publish uptime every second
        self.timer = self.create_timer(1.0, self.publish_uptime)

    def publish_uptime(self):
        # Calculate elapsed time since the node started
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9
        
        # Publish elapsed time
        self.publisher.publish(Float32(data=elapsed_time))
        #self.get_logger().info(f"Uptime: {elapsed_time:.2f} seconds")

def main(args=None):
    rclpy.init(args=args)
    node = UptimePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
