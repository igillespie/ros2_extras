import rclpy
import math
import tf2_ros
import numpy as np
import tf2_geometry_msgs
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Path, Odometry
from rclpy.action import ActionServer
from rclpy.action import ActionClient
from turn_in_place_interfaces.action import Turn
from move_to_point_interfaces.action import MoveToPoint
from move_to_point_interfaces.srv import PlanPath
from transforms3d.euler import quat2euler
from rclpy.executors import MultiThreadedExecutor
from enum import Enum
import time


### ros2 action send_goal /move_to_point move_to_point_interfaces/action/MoveToPoint "{target_x: 0.4, target_y: -0.05}" --feedback 

class MoveToPointStatus(Enum):
    SUCCESS = 1
    FAILURE = 2
    IN_PROGRESS = 3
    TURN_IN_PLACE_IN_PROGRESS = 4
    TIMEOUT = 5

class MoveToPointActionServer(Node):
    def __init__(self):
        super().__init__('move_to_point')

        # Transform listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Action server
        self.action_server = ActionServer(
            self,
            MoveToPoint,
            'move_to_point',
            self.execute_callback
        )
        

        self.turn_action_client = ActionClient(self, Turn, 'turn_in_place')
        self.turn_in_progress = False

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_intuitive', 10)
        self.path_publisher = self.create_publisher(Path, "/path", 10)

        # Odometry subscription
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)

        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.waypoints = []
        self.moving = False
        self.last_published_twist = None
        self.have_done_turn_in_place = False

    def odom_callback(self, msg):
        """Update current position from odometry data."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # self.get_logger().info(f"Odom callback: ({self.current_x:.3f}, {self.current_y:.3f})")
        # Convert quaternion to Euler angles
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = quat2euler([orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])
        self.current_yaw = yaw

    async def execute_callback(self, goal_handle):
        """Executes the MoveToPoint action."""
        self.get_logger().info(f"Received goal: x={goal_handle.request.target_x}, y={goal_handle.request.target_y}")

        # Call path planning service
        path_client = self.create_client(PlanPath, 'plan_path')

        while not path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for PathPlanner service...")

        #Clear this out for the new command we received
        self.have_done_turn_in_place = False

        request = PlanPath.Request()
        request.target_x = goal_handle.request.target_x
        request.target_y = goal_handle.request.target_y

        future = path_client.call_async(request)
        response = await future  # This is now a proper async call!
        
        if future.result() is not None:
            self.waypoints = future.result().waypoints
            num_waypoints = len(self.waypoints)

            if num_waypoints > 0:
                first_wp = self.waypoints[0]
                last_wp = self.waypoints[-1]
                self.get_logger().info(
                    f"Received {num_waypoints} waypoints. "
                    f"First: ({first_wp.position.x:.3f}, {first_wp.position.y:.3f}), "
                    f"Last: ({last_wp.position.x:.3f}, {last_wp.position.y:.3f})"
                )
            else:
                self.get_logger().warn("⚠️ Received an empty waypoint list!")

        else:
            self.get_logger().error("Failed to call PathPlanner service.")
            goal_handle.abort()
            return MoveToPoint.Result(success=False)

        # Publish path for visualization
        self.publish_path(self.waypoints)
        # Just for testing how the path looks
        # goal_handle.succeed()
        # move = MoveToPoint.Result()
        # move.success = True
        # return move
        # Start movement
        self.moving = True

        next_waypoint = self.waypoints.pop(0)
        next_x, next_y = next_waypoint.position.x, next_waypoint.position.y  # Take first waypoint and remove it
        self.move_to(next_x, next_y)
       
        move = MoveToPoint.Result()
        while rclpy.ok():
            status = self.follow_waypoints(goal_handle)
            # self.get_logger().info(f"Status callback is {status}.")
            match status:
                case MoveToPointStatus.SUCCESS:
                    self.stop_movement()
                    goal_handle.succeed()
                    move.success = True
                    break
                case MoveToPointStatus.FAILURE:
                    goal_handle.abort()
                    self.stop_movement()
                    move.success = False
                    break
                case MoveToPointStatus.TURN_IN_PLACE_IN_PROGRESS:
                    time.sleep(1/3)
                case MoveToPointStatus.IN_PROGRESS:
                    time.sleep(1/3) #make sure we collect some odometry and velocity data before calculating again
                case MoveToPointStatus.TIMEOUT:
                    goal_handle.abort()
                    self.stop_movement()
                    move.success = False
                    break
                case _:
                    self.get_logger().warn("Unknown status.")
                    goal_handle.abort()
                    self.stop_movement()
                    move.success = False
                    break
            
        return move

       
    def move_to(self, point_in_odom_x: float, point_in_odom_y: float):
        
        # Compute absolute target position
        self.target_x = point_in_odom_x
        self.target_y = point_in_odom_y

        self.moving = True
        self.get_logger().info(f"Moving towards absolute target: ({self.target_x:.3f}, {self.target_y:.3f})")


    def check_position(self) -> MoveToPointStatus:
        """Periodically check if the rover has reached its target and adjust movement."""

        self.get_logger().info(f"Current Position: x={self.current_x:.5f}, y={self.current_y:.5f}")
        self.get_logger().info(f"Target Position:  x={self.target_x:.5f}, y={self.target_y:.5f}")


        # ✅ Check if goal is reached
        if self.check_goal_reached():

            if self.waypoints:  # ✅ If more waypoints exist, continue moving
                next_waypoint = self.waypoints.pop(0)
                next_x, next_y = next_waypoint.position.x, next_waypoint.position.y
                self.get_logger().info(f"Continuing to next waypoint: ({next_x:.2f}, {next_y:.2f})")
                self.move_to(next_x, next_y)
            else:
                self.moving = False
                self.stop_movement()
                return MoveToPointStatus.SUCCESS
        else: 
            
            #self.publish_twist()

            dx = self.target_x - self.current_x
            dy = self.target_y - self.current_y

            target_angle = math.atan2(dy, dx)
            angular_error = target_angle - self.current_yaw

            # Normalize angular error to [-pi, pi]
            angular_error = (angular_error + math.pi) % (2 * math.pi) - math.pi
            self.get_logger().info(f"Yaw difference: {angular_error:.3f}")
            # if abs(angular_error) > 0.2:
            if self.have_done_turn_in_place is not True and abs(angular_error) > 0.2 and self.turn_in_progress is not True:
                self.have_done_turn_in_place = True
                self.stop_movement()
                self.command_turn_in_place(angular_error)
                return MoveToPointStatus.TURN_IN_PLACE_IN_PROGRESS
            else:
                self.have_done_turn_in_place = True #We only allow a turn in place on the first time through, so if it didn't meet conditions above...
                if self.turn_in_progress is not True:
                    self.publish_twist()

        return MoveToPointStatus.IN_PROGRESS

    def follow_waypoints(self, goal_handle) -> MoveToPointStatus:
        """Navigates through waypoints."""
        
        status = self.check_position()

        feedback = MoveToPoint.Feedback()
        feedback.current_x = self.current_x
        feedback.current_y = self.current_y
        feedback.current_yaw = self.current_yaw
        goal_handle.publish_feedback(feedback)

        return status

    
    def command_turn_in_place(self, angular_error):
        """Sends a non-blocking action request to the turn_in_place action server."""

        if self.turn_in_progress:
            self.get_logger().warn("⚠️ Turn already in progress, ignoring new turn request.")
            return

        self.turn_in_progress = True  # ✅ Mark as turning
        target_angle = math.degrees(angular_error)

        self.get_logger().info(f"🌀 Sending turn command: {target_angle:.3f}°")

        if not self.turn_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("⚠️ Turn action server not available!")
            self.turn_in_progress = False  # ✅ Reset flag
            return

        goal_msg = Turn.Goal()
        goal_msg.target_angle = target_angle

        # Send the goal asynchronously
        send_goal_future = self.turn_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.turn_goal_response_callback)

    def turn_goal_response_callback(self, future):
        """Handles response after sending a turn goal."""
        
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("🚫 Turn goal was rejected.")
            self.turn_in_progress = False  # ✅ Reset flag
            return

        self.get_logger().info("✅ Turn goal accepted, waiting for completion...")

        # Get result asynchronously
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.turn_result_callback)

    def turn_result_callback(self, future):
        """Handles the completion of the turn action."""
        
        result = future.result().result
        if result and result.success:
            self.get_logger().info("✅ Turn completed successfully.")
        else:
            self.get_logger().warn("⚠️ Turn action failed.")

        self.turn_in_progress = False  # ✅ Allow new turn requests

    def publish_twist(self):
        # Compute differences and overall distance
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)

        # Compute the desired heading
        target_angle = math.atan2(dy, dx)
        angular_error = target_angle - self.current_yaw

        # Normalize the angular error to [-pi, pi]
        angular_error = (angular_error + math.pi) % (2 * math.pi) - math.pi

        # Compute a lateral error scale: as the lateral difference decreases, scale down the turn.
        # Here, we assume that if |dy| is above a certain threshold, we use full turn gain,
        # and if |dy| is below that threshold, we scale down proportionally.
        lateral_error = abs(dy)
        threshold = 0.1  # For example, 0.1 m lateral error gives full correction.
        lateral_scale = min(1.0, lateral_error / threshold)
        
        # Use a proportional gain for angular correction.
        Kp_ang = 1.0  # You can tune this gain as needed.
        angular_speed = Kp_ang * angular_error * lateral_scale

        # Optionally, if the angular error is large, you might want to halt forward motion.
        min_speed = 0.075
        if abs(angular_error) > 0.3:  # If error > 0.3 rad (~17°)
            linear_speed = min_speed
        else:
            max_speed = 0.2
            
            linear_speed = max(min_speed, min(max_speed, distance * 0.5))
        
        # Cap the angular velocity to a maximum value.
        max_angular_speed = 0.8
        angular_speed = max(-max_angular_speed, min(max_angular_speed, angular_speed))

        twist_msg = Twist()
        twist_msg.linear.x = linear_speed
        twist_msg.angular.z = angular_speed

        self.cmd_vel_pub.publish(twist_msg)
        self.last_published_twist = twist_msg

        self.get_logger().info(
            f"Publishing Twist: Linear X={linear_speed:.5f}, Angular Z={angular_speed:.5f} "
            f"(angular_error: {angular_error:.3f} rad, lateral_scale: {lateral_scale:.3f})"
        )


    def check_goal_reached(self, x_error=0.05, y_error=0.1):
        """Return True if the robot is close to the target, otherwise False."""
        distance = math.sqrt((self.target_x - self.current_x) ** 2 + (self.target_y - self.current_y) ** 2)
        if distance < 0.05:  # Only returns True/False, no stopping logic
            return True
        elif self.last_published_twist is not None:
            velocity_x = self.last_published_twist.linear.x  # Forward/backward speed
            velocity_y = self.last_published_twist.linear.y  # Sideways speed (if applicable)

            # Check if we've overshot the target
            overshot_x = (self.current_x > self.target_x and velocity_x > 0) or \
                        (self.current_x < self.target_x and velocity_x < 0)

            overshot_y = (self.current_y > self.target_y and velocity_y > 0) or \
                        (self.current_y < self.target_y and velocity_y < 0)
            
            # Compute positional errors
            x_within_tolerance = abs(self.current_x - self.target_x) <= x_error
            y_within_tolerance = abs(self.current_y - self.target_y) <= y_error

            if (overshot_x and y_within_tolerance) or (overshot_y and x_within_tolerance):
                self.get_logger().info("Reached target position, stopping movement.")
                return True
            else:
                return False
        else:
            return False

    def stop_movement(self):
        """Stops the rover."""
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)

    # def quaternion_from_euler(self, roll, pitch, yaw):

    #     # qx, qy, qz, qw = euler2quat(0, 0, yaw)  # roll, pitch = 0
    #     # return Quaternion(x=qx, y=qy, z=qz, w=qw)

    #     qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) \
    #         - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    #     qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) \
    #         + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    #     qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) \
    #         - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    #     qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) \
    #         + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

    #     quat = Quaternion()
    #     quat.x = qx
    #     quat.y = qy
    #     quat.z = qz
    #     quat.w = qw

    #     return quat

    def publish_path(self, waypoints):
        """Publishes path to /path topic."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "odom"

        for waypoint in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header  # Inherit timestamp and frame_id
            pose_stamped.pose = waypoint  # Directly use the Pose object

            path_msg.poses.append(pose_stamped)

        self.path_publisher.publish(path_msg)
        self.get_logger().info("Published path to /path.")

def main():
    rclpy.init()
    node = MoveToPointActionServer()
    try:
        # ✅ Use MultiThreadedExecutor to allow multiple threads to run
        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor=executor)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()