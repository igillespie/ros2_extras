import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from mast_control_interfaces.msg import PanTiltState  # Correct message import
import math

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')

        # Declare parameters
        self.declare_parameter('image_topic', '/camera_frames/compressed')
        self.declare_parameter('pan_tilt_topic', '/pan_tilt_state')
        self.declare_parameter('camera_height', 0.6)  
        self.declare_parameter('output_topic', '/visualized_path/compressed')

        self.declare_parameter('jpeg_quality', 100)  # Default JPEG quality
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        
        # Get parameter values
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.pan_tilt_topic = self.get_parameter('pan_tilt_topic').get_parameter_value().string_value
        self.camera_height = self.get_parameter('camera_height').get_parameter_value().double_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        # Set up subscribers
        self.image_sub = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            10
        )

        self.pan_tilt_angle = 150.0  # Default: Level
        self.pan_tilt_sub = self.create_subscription(
            PanTiltState,
            self.pan_tilt_topic,
            self.pan_tilt_callback,
            10
        )

        # Set up publisher for the processed image
        self.image_pub = self.create_publisher(CompressedImage, self.output_topic, 10)

    def pan_tilt_callback(self, msg: PanTiltState):
        """ Updates the pan and tilt angles from PanTiltState message. """
        self.pan_tilt_angle = msg.tilt_position  # Correct tilt field

    def image_callback(self, msg):
        """ Processes the incoming image, overlays projected path lines, and republishes it. """
        try:
            # Convert CompressedImage to OpenCV format (JPEG decoding)
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Get image dimensions
            height, width, _ = cv_image.shape

            # Calculate projected lines
            self.draw_projected_path(cv_image, width, height)

            # Convert OpenCV image back to ROS `CompressedImage` and publish
            self.publish_annotated_image(cv_image)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def draw_projected_path(self, img, width, height):
        """
        Draws the projected movement path using two vertical lines and a dynamic horizontal line.
        - **Starts at the bottom of the image**.
        - **Extends further when the camera tilts downward**.
        - **Shortens when the camera is level or pointing slightly up**.
        - **Skewed inward for perspective**.
        - **Lines attach cleanly at the top to the horizontal line**.
        """
        # Constants for rover width and projection scaling
        rover_width = 0.5  # Approximate width in meters
        max_distance = 2.0  # **Reduced max projection distance (was 3.0m)**
        min_distance = 0.7  # **Reduced min projection distance (was 1.0m)**
        skew_factor = 0.5  # **Keeps strong perspective effect**
        projection_factor_min = 0.2  # **Shorter lines when level**
        projection_factor_max = 0.5  # **More reasonable extension when tilted down**

        # Ensure tilt stays in expected range (130°-150°)
        tilt_clamped = max(130, min(150, self.pan_tilt_angle))

        # **Adjust forward projection dynamically based on tilt angle**
        projection_factor = projection_factor_max - ((tilt_clamped - 130) / 20.0) * (projection_factor_max - projection_factor_min)

        # **Calculate top Y limit**
        top_y = int(height - (height * projection_factor))  # Adjusted based on tilt

        # **Bottom (wide) positions for the vertical lines**
        left_bottom_x = int((width / 2) - (rover_width * width / 2))  
        right_bottom_x = int((width / 2) + (rover_width * width / 2))

        # **Top (narrowed) positions - Increased skew effect**
        left_top_x = int(left_bottom_x * (1 - skew_factor) + (width / 2) * skew_factor)
        right_top_x = int(right_bottom_x * (1 - skew_factor) + (width / 2) * skew_factor)

        # **Draw vertical lines (stopping at adjusted top_y)**
        cv2.line(img, (left_bottom_x, height), (left_top_x, top_y), (0, 255, 0), 2)
        cv2.line(img, (right_bottom_x, height), (right_top_x, top_y), (0, 255, 0), 2)

        # **Draw horizontal line (connects correctly to the top of vertical lines)**
        cv2.line(img, (left_top_x, top_y), (right_top_x, top_y), (0, 255, 0), 2)

    def publish_annotated_image(self, img):
        """ Converts the annotated OpenCV image to a CompressedImage message and publishes it. """
        try:
            # Convert OpenCV image to JPEG format
            _, buffer = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            compressed_msg = CompressedImage()
            compressed_msg.format = "jpeg"
            compressed_msg.data = buffer.tobytes()

            # Publish the compressed image
            self.image_pub.publish(compressed_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to convert and publish image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()