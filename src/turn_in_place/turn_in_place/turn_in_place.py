import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from turn_in_place_interfaces.action import Turn
from transforms3d.euler import quat2euler
from math import radians, atan2, copysign
import math
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer

from enum import Enum

class TurnStatus(Enum):
    SUCCESS = 1
    FAILURE = 2
    IN_PROGRESS = 3
    TIMEOUT = 4


# PUBLISH THIS TO TEST: ros2 action send_goal /turn_in_place turn_in_place_interfaces/action/Turn "{target_angle: 15.0}" 

class TurnInPlaceNode(Node):
    def __init__(self):
        super().__init__('turn_in_place')

        self.publisher = self.create_publisher(Twist, 'cmd_vel_intuitive', 10)
        
        self._action_server = ActionServer(
            self,
            Turn,
            'turn_in_place',
            self.execute_turn_callback
        )

        self.latest_pose = None
        self.last_published_cmd_vel = None
        self.last_published_time = None

        self.current_velocity_from_odom_differential = 0.0
        self.mininum_speed_to_turn = 0.0

        self.latest_drive_state= None

        self.current_yaw = None
        self.target_yaw = None
        self.yaw_tolerance = 0.1
        self.turn_start_time = None
        self.timeout = 30.0  # 10 seconds timeout for turn completion

        # Persistent subscription to odometry
        self.pose_subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.update_pose,
            10
        )

        self.last_pose_log = None

        self.drive_state_subscription = self.create_subscription(
            JointState,
            '/drive_state',
            self.update_drive_state,
            10
        )

        self.get_logger().info('TurnInPlaceNode action server ready and listening for turn commands.')

    def extract_yaw(self, orientation: Quaternion):
        """
        Extract the yaw (rotation around Z-axis) from a quaternion.
        """

        # Convert quaternion to euler angles
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]  # Note the order for transforms3d
        # _, _, yaw = quat2euler(quaternion)
        # return yaw
        _, _, yaw = self.euler_from_quaternion(quaternion)
        return yaw

    def euler_from_quaternion(self, quat):
        """
        Convert a quaternion into Euler angles (roll, pitch, yaw).
        """
        x, y, z, w = quat

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def normalize_angle(self, angle):
        """
        Normalize angle to the range [-π, π].
        """
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def latest_angular_velocity(self):
        return self.current_velocity_from_odom_differential
        # return self.latest_pose.twist.twist.angular.z

    def update_drive_state(self, msg):
        self.latest_drive_state = msg

    def all_wheels_moving(self):

        if self.latest_drive_state is None:
            self.get_logger().warn("latest_drive_state is None. Cannot determine wheel velocities.")
            return False

        epsilon = 0.0001
        count_over_threshold = 0
        for velocity in self.latest_drive_state.velocity:  # Assuming `latest_drive_state` has a `velocity` attribute
            if abs(velocity) >= epsilon:
                count_over_threshold += 1
                
        # All wheels have significant velocities
        return count_over_threshold >= 3 #3 wheels is good enough


    def update_pose(self, msg):

        current_time = self.get_clock().now().to_msg()

        if self.latest_pose is not None:
            
            # Extract the timestamp and yaw from the latest pose
            pose_time = self.latest_pose.header.stamp
            pose_yaw = self.extract_yaw(msg.pose.pose.orientation)

            # Calculate time difference
            
            time_difference = (current_time.sec + current_time.nanosec * 1e-9) - (pose_time.sec + pose_time.nanosec * 1e-9)

            # Calculate angular velocity if time difference is valid
            if time_difference > 0:
                angular_velocity = (self.current_yaw - pose_yaw) / time_difference
                self.current_velocity_from_odom_differential = angular_velocity


        self.latest_pose = msg
        self.current_yaw = self.extract_yaw(msg.pose.pose.orientation)

        # Convert ROS2 time message to seconds for comparison
        now_sec = current_time.sec + current_time.nanosec * 1e-9
        last_log_sec = self.last_pose_log.sec + self.last_pose_log.nanosec * 1e-9 if self.last_pose_log else None

        # Log yaw every second
        # if self.last_pose_log is None or (now_sec - last_log_sec) >= 1.0:
        #     self.get_logger().info(f"Pose yaw = {self.current_yaw:.3f} radians.")
        #     self.last_pose_log = self.get_clock().now().to_msg()

    def calculate_target_yaw(self, angle_degrees):
        """
        Calculate the target yaw based on the current yaw and the desired turn angle.
        """
        return self.normalize_angle(self.current_yaw + radians(angle_degrees))

    def calculate_yaw_error(self, target_yaw, current_yaw):
        """
        Calculate the smallest angular difference to the target yaw.
        """
        yaw_error = self.normalize_angle(target_yaw - current_yaw)

        # Special case: If yaw_error is exactly π or -π, decide the turning direction
        if math.isclose(abs(yaw_error), math.pi, abs_tol=1e-6):
            # Force a consistent direction for π (you can adjust based on your preference)
            yaw_error = -math.pi if yaw_error > 0 else math.pi

        return yaw_error

    def calculate_angular_speed(self, yaw_error, midpoint):
        """
        Calculate angular speed with ramp-up and ramp-down based on midpoint logic.
        Also dynamically adjusts speed based on the remaining yaw error.

        This method will only get called at the rate we want to publish to cmd_vel_intuitive.
        """
        # Constants
        max_speed = 0.9  # Maximum angular speed in radians/sec
        min_speed = 0.1  # Minimum angular speed to prevent stalling
        epsilon = 0.05  # Threshold to treat previous speed as effectively zero
        scale_factor = 0.1

        # Previous speed for smoother ramping
        previous_speed = self.last_published_cmd_vel.angular.y if self.last_published_cmd_vel else 0.0  # We publish to y for turn in place

        self.get_logger().debug(f"PREV speed={previous_speed:.2f}")

        scaled_speed = max(abs(previous_speed), abs(min_speed))

        if not self.all_wheels_moving():
            scaled_speed = scaled_speed + scale_factor
            self.mininum_speed_to_turn = scaled_speed
            self.get_logger().debug("RAMPING")
        else:
            # We are moving
            latest_vel = self.latest_angular_velocity()

            # Handle divide by zero case
            if abs(latest_vel) < 0.00001:
                time_to_reach_target = float('inf')  # Large number, essentially meaning we can't determine it
                self.get_logger().warn("Latest velocity is too low, setting time_to_reach_target to infinity.")
            else:
                time_to_reach_target = abs(yaw_error) / abs(latest_vel)

            self.get_logger().debug(f"TIME to target: {time_to_reach_target:.2f}s at latest ang speed={latest_vel:.4f} rad/s")

            if abs(yaw_error) > abs(midpoint - (midpoint / 2.0)) and time_to_reach_target >= 500.0:  # Ramp up before midpoint, adding huge check here to try slowing things down
                if math.isclose(previous_speed, 0.0, abs_tol=epsilon) or not self.all_wheels_moving():
                    scaled_speed = previous_speed + scale_factor
                elif latest_vel <= 0.25:  # This is real-world velocity, don't go faster than that
                    scaled_speed = scaled_speed + scale_factor / 4.0
                self.get_logger().debug(f"BEFORE midpoint: yaw_error={yaw_error:.2f}, scaled_speed={scaled_speed:.2f}")
            else:  # Ramp down after passing midpoint
                if math.isclose(previous_speed, 0.0, abs_tol=epsilon) or not self.all_wheels_moving():
                    scaled_speed = previous_speed + scale_factor
                    self.get_logger().debug("Might have stopped")
                else:
                    scaled_speed = self.mininum_speed_to_turn
                self.get_logger().debug(f"AFTER midpoint: yaw_error={yaw_error:.2f}, scaled_speed={scaled_speed:.2f}")

        # Clamp speed to ensure it's within the allowed range
        scaled_speed = max(min(scaled_speed, max_speed), min_speed)

        # Maintain the sign of yaw_error for direction
        angular_speed = copysign(scaled_speed, yaw_error)
        self.get_logger().debug(f"Calculated angular speed: angular_speed={angular_speed:.2f}, previous_speed={previous_speed:.2f}")
        
        return angular_speed

    def stop_robot(self):
        """
        Publish zero velocities to stop the robot.
        """
        twist = Twist()
        self.publisher.publish(twist)

    async def execute_turn_callback(self, goal_handle):
        """
        Handle the turn command by initializing the target yaw and starting monitoring.
        """
        
        if self.current_yaw is None:
            self.get_logger().warn("Cannot execute turn: Waiting for odometry data.")
            goal_handle.abort()
            turn = Turn.Result()
            turn.success = False
            return turn

        #clear out any cached values
        self.current_velocity_from_odom_differential = 0.0
        self.mininum_speed_to_turn = 0.0
        self.last_published_cmd_vel = None

        # Set target yaw
        self.target_yaw = self.calculate_target_yaw(goal_handle.request.target_angle)
        self.turn_start_time = self.get_clock().now().nanoseconds / 1e9

        

        total_yaw_error = self.calculate_yaw_error(self.target_yaw, self.current_yaw)
        self.midpoint = total_yaw_error / 2.0

        #hand tune yaw tolerance, smaller turns, smaller tolerance
        if abs(total_yaw_error < 0.7854): #45˚
            self.yaw_tolerance = 0.05
        else:
            self.yaw_tolerance = 0.1
       
       # Log the current and target yaw
        self.get_logger().info(
            f"Received turn command: degrees={goal_handle.request.target_angle} current_yaw={self.current_yaw:.2f} radians, "
            f"target_yaw={self.target_yaw:.2f} radians."
        )

        turn = Turn.Result()
        while rclpy.ok():
            status = self.monitor_turn(goal_handle)
            # self.get_logger().info(f"Status callback is {status}.")
            match status:
                case TurnStatus.SUCCESS:
                    self.stop_robot()
                    goal_handle.succeed()
                    turn.success = True
                    break
                case TurnStatus.FAILURE:
                    goal_handle.abort()
                    self.stop_robot()
                    turn.success = False
                    break
                case TurnStatus.IN_PROGRESS:
                    time.sleep(1/2) #make sure we collect some odometry and velocity data before calculating again
                case TurnStatus.TIMEOUT:
                    goal_handle.abort()
                    self.stop_robot()
                    turn.success = False
                    break
                case _:
                    self.get_logger().warn("Unknown status.")
                    goal_handle.abort()
                    self.stop_robot()
                    turn.success = False
                    break
            
        return turn

    def send_feedback(self, goal_handle):
        """ Send feedback to the action client. """

        feedback_msg = Turn.Feedback()

        # ✅ Ensure `self.latest_pose` is a Pose message
        if self.latest_pose and isinstance(self.latest_pose, Odometry):
            feedback_msg.current_pose = self.latest_pose.pose.pose
        else:
            # ✅ If latest_pose is None or invalid, send a default Pose
            self.get_logger().warn("latest_pose is not set or invalid, sending default Pose.")
            feedback_msg.current_pose = Pose()  # Sends an empty Pose message instead of crashing

        goal_handle.publish_feedback(feedback_msg)

    def monitor_turn(self, goal_handle) -> TurnStatus:
        """
        Monitor the turn and stop the robot once the target is reached.
        """
        # self.get_logger().info("Monitor turn")
        if self.current_yaw is None or self.target_yaw is None:
            self.get_logger().warn("Pose data unavailable. Skipping monitoring step.")
            return TurnStatus.ERROR

        # Check timeout
        current_time = self.get_clock().now().nanoseconds / 1e9

        if (current_time - self.turn_start_time) > self.timeout:
            self.get_logger().warn("Turn timed out. Stopping robot.")
            return TurnStatus.TIMEOUT
            

        # Calculate yaw error
        yaw_error = self.calculate_yaw_error(self.target_yaw, self.current_yaw)

        # Log the current yaw and yaw error
        self.get_logger().info(
            f"Monitoring turn: current_yaw={self.current_yaw:.2f} radians, "
            f"target_yaw={self.target_yaw:.2f} radians, "
            f"yaw_error={yaw_error:.2f} radians."
        )

        if abs(yaw_error) > self.yaw_tolerance:
            # Initialize last_published_time if not set
            if not hasattr(self, "last_published_time") or self.last_published_time is None:
                self.last_published_time = self.get_clock().now().nanoseconds / 1e9

            last_published_time = self.last_published_time  # Retrieve the last published time

            # Calculate the time difference in seconds
            time_since_last_publish = current_time - last_published_time

            if time_since_last_publish >= 0.1:  # Ensure at least 0.1 seconds (10 Hz) between publishes
                # Calculate and publish angular velocity
                angular_speed = self.calculate_angular_speed(yaw_error, self.midpoint)
                twist = Twist()
                twist.angular.y = -angular_speed  # Use angular.y for Ackerman steering

                self.last_published_cmd_vel = twist  # Cache the most recent message
                self.publisher.publish(twist)
                self.last_published_time = current_time  # Update last published time
                self.send_feedback(goal_handle)
                
                self.get_logger().debug(
                    f"Published twist: angular_speed={angular_speed:.2f}, time_since_last_publish={time_since_last_publish:.2f}s"
                )
                return TurnStatus.IN_PROGRESS
            else:
                self.get_logger().debug(
                    f"Skipped publishing: time_since_last_publish={time_since_last_publish:.2f}s (10 Hz limit)"
                )
                return TurnStatus.IN_PROGRESS
        else:
            # Target reached
            self.get_logger().info(f"Turn complete. Robot stopped at {self.current_yaw}")
            return TurnStatus.SUCCESS


def main(args=None):
    rclpy.init(args=args)
    node = TurnInPlaceNode()

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