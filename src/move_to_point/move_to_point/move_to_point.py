import rclpy
import math
import tf2_ros
import numpy as np
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PoseStamped, PointStamped, Twist
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D 
from transforms3d.euler import quat2euler


class MoveToPoint(Node):
    def __init__(self):
        super().__init__('move_to_point')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher for movement commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_intuitive', 10)

        # Subscriber to /odometry/filtered
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)

        self.nav_sub = self.create_subscription(Pose2D, '/move_to_target', self.nav_callback, 10)
        # Current robot position & orientation
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0  # Robot's heading in radians

        # Target path (waypoints)
        self.path = [(3, 1)]  # Example final target, can be updated
        self.lookahead_distance = 0.5  # How far ahead to select a target point

        # Target position
        self.target_x = None #transformed into odom frame
        self.target_y = None #transformed into odom frame
        self.moving = False
        self.timer = None
        self.last_published_twist = None

    def odom_callback(self, msg):
        """Update current position from odometry data."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convert quaternion to Euler angles to get yaw
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = quat2euler([orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])
        self.current_yaw = yaw  # Use yaw for heading

    def elliptical_arc(self, x_start, y_start, x_end, y_end, num_points, minor_axis_ratio=0.15, convex=False):
        """
        Calculate an elliptical arc connecting two points.
        
        The ellipse is defined with:
        - The center as the midpoint of the two points.
        - The semi-major axis (a) as half the distance between the points.
        - The semi-minor axis (b) set to a percentage of a.
        - The ellipse rotated so that its major axis aligns with the two points.

        Parameters:
            x_start, y_start (float): Start point coordinates.
            x_end, y_end (float): End point coordinates.
            num_points (int): Number of points along the arc.
            minor_axis_ratio (float): Defines the ellipse's flatness (default 0.15).
            convex (bool): If True, generate the convex variant of the arc.

        Returns:
            np.ndarray: An array of (x, y) points forming the elliptical arc.
        """
        # Define the start and end points as numpy arrays.
        p_start = np.array([x_start, y_start], dtype=np.float64)
        p_end = np.array([x_end, y_end], dtype=np.float64)
        
        # Compute the center (midpoint)
        center = (p_start + p_end) / 2
        
        # Compute the semi-major axis (a) as half the distance between the points.
        a = np.linalg.norm(p_end - p_start) / 2

        # Compute the semi-minor axis (b).
        b = a * minor_axis_ratio

        # Compute the rotation angle.
        dx, dy = p_end - center
        theta = np.arctan2(dy, dx)

        # Create the rotation matrix.
        R = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta),  np.cos(theta)]
        ])
        
        # Adjust t range for concave vs. convex arc
        if convex:
            t = np.linspace(np.pi, 2 * np.pi, num_points)  # Convex
        else:
            t = np.linspace(np.pi, 0, num_points)  # Concave (default)

        # Compute the elliptical arc in local coordinates.
        x_local = a * np.cos(t)
        y_local = b * np.sin(t)

        # Rotate and translate points.
        local_points = np.vstack((x_local, y_local))
        global_points = R @ local_points + center.reshape(2, 1)

        # Extract x and y values
        arc_x = global_points[0, :]
        arc_y = global_points[1, :]

        # Ensure x-values remain at or above x_start
        arc_x = np.maximum(arc_x, x_start)

        # Ensure y-values remain between y_start and y_end
        y_min, y_max = min(y_start, y_end), max(y_start, y_end)
        arc_y = np.clip(arc_y, y_min, y_max)

        return np.column_stack((arc_x, arc_y))

    
    
    def nav_callback(self, msg):
        """Receives a target position from another node."""
        self.get_logger().info(f"Received navigation command: x={msg.x}, y={msg.y}")

        try:
            transform = self.tf_buffer.lookup_transform(
                "odom", "base_link", rclpy.time.Time()
            )
        except tf2_ros.LookupException:
            self.get_logger().warn("Could not get transform from base_link to odom!")
            return

        # Create a point in the robot's local frame (base_link)
        point_in_base = PointStamped()
        point_in_base.header.frame_id = "base_link"
        point_in_base.header.stamp = self.get_clock().now().to_msg()
        point_in_base.point.x = msg.x  # Forward/backward
        point_in_base.point.y = msg.y  # Left/right
        point_in_base.point.z = 0.0

        try:
            # ✅ Now transform it safely
            point_in_odom = tf2_geometry_msgs.do_transform_point(point_in_base, transform)
            target_x, target_y = point_in_odom.point.x, point_in_odom.point.y
        except Exception as e:
            self.get_logger().warn(f"Failed to transform point: {str(e)}")
            return

        # ✅ Ensure target_x and target_y exist before using them
        if target_x is None or target_y is None:
            self.get_logger().warn("Transformation failed, target_x or target_y is None")
            return

        # Compute the Euclidean distance
        distance = math.sqrt((target_x - self.current_x) ** 2 + (target_y - self.current_y) ** 2)

        # If distance > 0.5m, break it into smaller waypoints
        step_size = 0.2  # Each segment length
        num_steps = max(1, int(distance / step_size))  # Ensure at least one step
        if num_steps == 1:
            num_steps = 2

        self.path.clear()  # Clear existing path

        self.get_logger().info(f"Will plot from cur_x={self.current_x}, cur_y={self.current_y} to target_x={target_x}, target_y={target_y}")
        self.path = self.elliptical_arc(self.current_x, self.current_y, target_x, target_y, num_steps, 0.15, True).tolist()
        # self.path = self.generate_arc_waypoints(self.current_x, self.current_y, target_x, target_y)

        self.get_logger().info(f"Generated {len(self.path)} waypoints for smooth navigation: {self.path}")
        self.get_logger().info(f"Final target position: ({target_x}, {target_y})")

    
        next_x, next_y = self.path.pop(0)  # Take first waypoint and remove it
        self.move_to(next_x, next_y)

    def move_to(self, point_in_odom_x: float, point_in_odom_y: float):
        
        # Compute absolute target position
        self.target_x = point_in_odom_x
        self.target_y = point_in_odom_y

        self.moving = True
        self.get_logger().info(f"Moving towards absolute target: ({self.target_x:.3f}, {self.target_y:.3f})")

        # Start the movement loop
        if self.timer is None:
            self.timer = self.create_timer(0.1, self.check_position)
            self.get_logger().info("Movement timer started.")


    def check_position(self):
        """Periodically check if the rover has reached its target and adjust movement."""
        if not self.moving:
            return

        self.get_logger().info(f"Current Position: x={self.current_x:.5f}, y={self.current_y:.5f}")
        self.get_logger().info(f"Target Position:  x={self.target_x:.5f}, y={self.target_y:.5f}")


        # ✅ Check if goal is reached
        if self.check_goal_reached():

            if self.path:  # ✅ If more waypoints exist, continue moving
                next_x, next_y = self.path.pop(0)
                self.get_logger().info(f"Continuing to next waypoint: ({next_x:.2f}, {next_y:.2f})")
                self.move_to(next_x, next_y)
            else:
                self.moving = False
                self.stop_movement()
                self.get_logger().info("Goal reached!")
                self.destroy_timer(self.timer)  # ✅ Stop the timer
                self.timer = None
        else: 
            # ✅ Call publish_twist() to handle movement
            self.publish_twist()

    def publish_twist(self):
        """Calculates movement direction and publishes the Twist message."""
        # Compute target in global frame
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)

        target_angle = math.atan2(dy, dx)
        angular_error = target_angle - self.current_yaw

        # Normalize angular error to [-pi, pi]
        angular_error = (angular_error + math.pi) % (2 * math.pi) - math.pi

        # ✅ Adjust Linear Speed
        max_speed = 0.2
        min_speed = 0.075
        linear_speed = max(min_speed, min(max_speed, distance * 0.5))  

        # ✅ Smooth Angular Speed Adjustment
        max_angular_speed = 0.8  # Keep a reasonable max
        if abs(angular_error) < 0.02:  # Ignore very small errors
            angular_speed = 0.0
        else:
            # ✅ Use a smooth scaling function
            scaling_factor = abs(angular_error) / (math.pi / 2)  # Normalize to range [0,1]
            scaling_factor = min(1.0, scaling_factor)  # Cap at 1

            angular_speed = max(-max_angular_speed, min(max_angular_speed, angular_error * scaling_factor * 0.8))  # Apply slight dampening

        # ✅ Create and publish Twist message
        twist_msg = Twist()
        twist_msg.linear.x = linear_speed  
        twist_msg.angular.z = angular_speed  

        self.get_logger().info(f"Publishing Twist: Linear X={twist_msg.linear.x:.5f}, Angular Z={twist_msg.angular.z:.5f}")

        self.cmd_vel_pub.publish(twist_msg)
        self.last_published_twist = twist_msg


    def check_goal_reached(self):
        """Return True if the robot is close to the target, otherwise False."""
        distance = math.sqrt((self.target_x - self.current_x) ** 2 + (self.target_y - self.current_y) ** 2)
        if distance < 0.05:  # ✅ Only returns True/False, no stopping logic
            return True
        elif self.last_published_twist is not None:
            velocity_x = self.last_published_twist.linear.x  # Forward/backward speed
            velocity_y = self.last_published_twist.linear.y  # Sideways speed (if applicable)

            # ✅ Check if we've overshot the target
            overshot_x = (self.current_x > self.target_x and velocity_x > 0) or \
                        (self.current_x < self.target_x and velocity_x < 0)

            overshot_y = (self.current_y > self.target_y and velocity_y > 0) or \
                        (self.current_y < self.target_y and velocity_y < 0)
            
            if overshot_x or overshot_y:
                self.get_logger().info("We overshot the distance, will stop.")
                return True
            else:
                return False
        else:
            return False

    def stop_movement(self):
        """Send a stop command to the rover."""
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)


def main():
    rclpy.init()
    node = MoveToPoint()
    
    # Move to (1,1)
    # node.move_to(1.0, 1.0)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()