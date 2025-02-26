import rclpy
import numpy as np
import math
import tf2_ros
import tf2_geometry_msgs
from rclpy.node import Node
from move_to_point_interfaces.srv import PlanPath
from geometry_msgs.msg import Pose, PointStamped, Quaternion
from nav_msgs.msg import Odometry

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        # Service for handling path requests
        self.srv = self.create_service(PlanPath, 'plan_path', self.plan_path_callback)
        self.get_logger().info("✅ PathPlanner Service is ready.")

        # TF Buffer and Listener (For Transforming Points)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribe to /odometry/filtered to track robot position
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        # Store current position in odom frame
        self.current_x = 0.0
        self.current_y = 0.0

    def odom_callback(self, msg):
        """Updates current robot position from odometry data."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def plan_path_callback(self, request, response):
        """Generates a smooth path, transforming points from relative to odom frame."""
        self.get_logger().info(f"Received request: Move ({request.target_x}, {request.target_y})")

        # Transform the relative target point to odom frame
        target_x, target_y = self.transform_to_odom(request.target_x, request.target_y)
        if target_x is None:
            self.get_logger().error("❌ Failed to transform target to odom frame.")
            return response

        self.get_logger().info(f"Transformed to odom: ({target_x}, {target_y})")

        # Generate elliptical arc waypoints
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        num_points = 5
        if distance < 0.5:
            num_points = 2
        elif distance > 0.5 and distance < 1.5:
            num_points = 5
        else:
            num_points = 8

        do_convex = target_y > 0
        waypoints = self.elliptical_arc(self.current_x, self.current_y, target_x, target_y, num_points=num_points, minor_axis_ratio=0.15, convex=do_convex)

        # Convert waypoints to Pose2D messages
        for i in range(len(waypoints)):
            x, y = waypoints[i]
            last_yaw = None
            if i < len(waypoints) - 1:
                # Compute yaw as the angle to the next waypoint
                dx = waypoints[i + 1][0] - x
                dy = waypoints[i + 1][1] - y
                yaw = math.atan2(dy, dx)
                last_yaw = yaw
            elif last_yaw is not None:
                # For the last waypoint, keep the previous yaw
                yaw = last_yaw
            
            if yaw is not None:
                # Convert yaw to quaternion
                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.position.z = 0.0  # Assuming movement is in 2D space (z=0)
                pose.orientation = self.quaternion_from_euler(0, 0, yaw)

                response.waypoints.append(pose)  

        self.get_logger().info(f"✅ Generated {len(response.waypoints)} waypoints.")
        return response

    def transform_to_odom(self, rel_x, rel_y):
        """Transforms a point in the robot's frame (base_link) to the odom frame."""
        try:
            # Get the latest transform from base_link to odom
            transform = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time())

            # Create the point in base_link frame
            point_in_base = PointStamped()
            point_in_base.header.frame_id = "base_link"
            point_in_base.header.stamp = self.get_clock().now().to_msg()
            point_in_base.point.x = rel_x
            point_in_base.point.y = rel_y
            point_in_base.point.z = 0.0

            # Transform to odom frame
            point_in_odom = tf2_geometry_msgs.do_transform_point(point_in_base, transform)
            return point_in_odom.point.x, point_in_odom.point.y

        except tf2_ros.LookupException:
            self.get_logger().warn("⚠️ Could not get transform from base_link to odom.")
            return None, None
        except Exception as e:
            self.get_logger().error(f"❌ Transformation failed: {str(e)}")
            return None, None

    def elliptical_arc(self, x_start, y_start, x_end, y_end, num_points, minor_axis_ratio=0.15, convex=False):
        """Computes an elliptical arc between two points in odom space."""
        
        # ✅ If y_end == 0, return a straight line path
        p_start = np.array([x_start, y_start], dtype=np.float64)
        p_end = np.array([x_end, y_end], dtype=np.float64)
        
        # ✅ Compute delta values
        dx = x_end - x_start
        dy = y_end - y_start

        # self.get_logger().info(f"elliptical_arc() called with:")
        # self.get_logger().info(f"  Start: ({x_start:.6f}, {y_start:.6f})")
        # self.get_logger().info(f"  End:   ({x_end:.6f}, {y_end:.6f})")
        # self.get_logger().info(f"  dx: {dx:.6f}, dy: {dy:.6f}")

        # ✅ If dy is very small, generate a straight-line path
        if abs(dy) < 0.02:  # Threshold to ignore small deviations
            x_vals = np.linspace(x_start, x_end, num_points)
            y_vals = np.full_like(x_vals, y_start)  # Keep y constant
            return np.column_stack((x_vals, y_vals))

        p_start = np.array([x_start, y_start], dtype=np.float64)
        p_end = np.array([x_end, y_end], dtype=np.float64)
        center = (p_start + p_end) / 2
        a = np.linalg.norm(p_end - p_start) / 2
        b = a * minor_axis_ratio

        dx, dy = p_end - center
        theta = np.arctan2(dy, dx)
        R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

        t = np.linspace(np.pi, 2 * np.pi, num_points) if convex else np.linspace(np.pi, 0, num_points)
        x_local = a * np.cos(t)
        y_local = b * np.sin(t)

        global_points = R @ np.vstack((x_local, y_local)) + center.reshape(2, 1)
        return np.column_stack((global_points[0, :], global_points[1, :]))

    def quaternion_from_euler(self, roll, pitch, yaw):

        # qx, qy, qz, qw = euler2quat(0, 0, yaw)  # roll, pitch = 0
        # return Quaternion(x=qx, y=qy, z=qz, w=qw)

        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) \
            - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) \
            + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) \
            - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) \
            + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

        quat = Quaternion()
        quat.x = qx
        quat.y = qy
        quat.z = qz
        quat.w = qw

        return quat

def main():
    rclpy.init()
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()