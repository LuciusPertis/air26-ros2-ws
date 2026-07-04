#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64, Bool
from tf_transformations import quaternion_from_euler

class EnhancedWheelOdometryNode(Node):
    def __init__(self):
        super().__init__('enhanced_wheel_odometry_node')

        # Removed steering joints, keeping differential drive parameters
        self.declare_parameter('wheel_radius', 0.034) 
        self.declare_parameter('track_width', 0.1255) 
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('left_wheel_joint', 'base_back_left_wheel_joint')
        self.declare_parameter('right_wheel_joint', 'base_back_right_wheel_joint')
        self.declare_parameter('publish_rate', 50.0)
        
        # Enhanced filtering parameters
        self.declare_parameter('velocity_filter_alpha', 0.2)
        self.declare_parameter('max_wheel_acceleration', 3.0)
        self.declare_parameter('deadzone_threshold', 0.003)
        
        # Slip detection parameters
        self.declare_parameter('slip_detection_enabled', True)
        self.declare_parameter('slip_threshold_velocity', 0.05)
        self.declare_parameter('slip_threshold_acceleration', 2.0)
        self.declare_parameter('slip_recovery_time', 1.0)
        
        # Covariance parameters
        self.declare_parameter('base_position_variance', 0.005)
        self.declare_parameter('speed_variance_factor', 0.03)
        self.declare_parameter('acceleration_variance_factor', 0.02)

        # Get parameter values
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.track_width = self.get_parameter('track_width').get_parameter_value().double_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.left_wheel_joint = self.get_parameter('left_wheel_joint').get_parameter_value().string_value
        self.right_wheel_joint = self.get_parameter('right_wheel_joint').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.velocity_filter_alpha = self.get_parameter('velocity_filter_alpha').get_parameter_value().double_value
        self.max_wheel_acceleration = self.get_parameter('max_wheel_acceleration').get_parameter_value().double_value
        self.deadzone_threshold = self.get_parameter('deadzone_threshold').get_parameter_value().double_value
        
        self.slip_detection_enabled = self.get_parameter('slip_detection_enabled').get_parameter_value().bool_value
        self.slip_threshold_velocity = self.get_parameter('slip_threshold_velocity').get_parameter_value().double_value
        self.slip_threshold_acceleration = self.get_parameter('slip_threshold_acceleration').get_parameter_value().double_value
        self.slip_recovery_time = self.get_parameter('slip_recovery_time').get_parameter_value().double_value
        
        self.base_position_variance = self.get_parameter('base_position_variance').get_parameter_value().double_value
        self.speed_variance_factor = self.get_parameter('speed_variance_factor').get_parameter_value().double_value
        self.acceleration_variance_factor = self.get_parameter('acceleration_variance_factor').get_parameter_value().double_value

        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.last_wheel_pos = {'left': 0.0, 'right': 0.0}
        self.joint_names = []
        self.initialized = False
        
        self.current_wheel_vel = {'left': 0.0, 'right': 0.0}
        self.filtered_wheel_vel = {'left': 0.0, 'right': 0.0}
        self.last_wheel_vel = {'left': 0.0, 'right': 0.0}
        self.wheel_acceleration = {'left': 0.0, 'right': 0.0}
        
        self.slip_detected = False
        self.slip_start_time = None
        self.slip_confidence = 0.0
        
        self.velocity_kf_initialized = False
        self.velocity_state = np.zeros(4)
        self.velocity_covariance = np.eye(4) * 0.1
        self.process_noise = np.diag([0.01, 0.1, 0.01, 0.1])
        self.measurement_noise = np.diag([0.05, 0.05])
        
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        
        # Publishers and Subscribers
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odometry/odom', 10)
        self.slip_pub = self.create_publisher(Bool, '/wheel_odometry/slip_detected', 10)
        self.slip_confidence_pub = self.create_publisher(Float64, '/wheel_odometry/slip_confidence', 10)
        self.wheel_velocities_pub = self.create_publisher(Vector3, '/wheel_odometry/wheel_velocities', 10)
        
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_odometry)

        self.get_logger().info('Enhanced Differential Odometry Node Started')

    def initialize_velocity_kalman_filter(self, left_vel, right_vel):
        """Initialize Kalman filter for velocity estimation"""
        self.velocity_state = np.array([left_vel, 0.0, right_vel, 0.0])
        self.velocity_covariance = np.eye(4) * 0.1
        self.velocity_kf_initialized = True
        
    def predict_velocity_kalman_filter(self, dt):
        """Predict step of Kalman filter"""
        # State transition matrix F
        F = np.array([[1, dt, 0, 0],
                      [0, 1,  0, 0],
                      [0, 0,  1, dt],
                      [0, 0,  0, 1]])
        
        # Predict state and covariance
        self.velocity_state = F @ self.velocity_state
        self.velocity_covariance = F @ self.velocity_covariance @ F.T + self.process_noise * dt
        
    def update_velocity_kalman_filter(self, measured_left_vel, measured_right_vel):
        """Update step of Kalman filter"""
        # Measurement matrix H (we observe velocities directly)
        H = np.array([[1, 0, 0, 0],
                      [0, 0, 1, 0]])
        
        # Measurement
        z = np.array([measured_left_vel, measured_right_vel])
        
        # Innovation
        y = z - H @ self.velocity_state
        
        # Innovation covariance
        S = H @ self.velocity_covariance @ H.T + self.measurement_noise
        
        # Kalman gain
        K = self.velocity_covariance @ H.T @ np.linalg.inv(S)
        
        # Update state and covariance
        self.velocity_state = self.velocity_state + K @ y
        self.velocity_covariance = (np.eye(4) - K @ H) @ self.velocity_covariance
        
        # Extract accelerations
        self.wheel_acceleration['left'] = self.velocity_state[1]
        self.wheel_acceleration['right'] = self.velocity_state[3]

    def detect_wheel_slip(self, left_vel, right_vel, dt):
        """Advanced slip detection using multiple criteria"""
        if not self.slip_detection_enabled:
            return False
            
        current_time = self.get_clock().now()
        slip_indicators = []
        
        # 1. Velocity difference criterion
        expected_vel_diff = abs(self.current_angular_velocity * self.track_width / 2.0)
        actual_vel_diff = abs(left_vel - right_vel)
        vel_diff_ratio = actual_vel_diff / max(expected_vel_diff, 0.01)
        
        if vel_diff_ratio > 2.0:  # Actual difference is too large
            slip_indicators.append(0.3)
        
        # 2. Acceleration-based detection
        max_accel = max(abs(self.wheel_acceleration['left']), abs(self.wheel_acceleration['right']))
        if max_accel > self.slip_threshold_acceleration:
            slip_indicators.append(0.4)
            
        # 3. Velocity consistency check
        avg_vel = (left_vel + right_vel) / 2.0
        if avg_vel > 0.1:  # Only check when moving
            left_deviation = abs(left_vel - avg_vel) / avg_vel
            right_deviation = abs(right_vel - avg_vel) / avg_vel
            
            if left_deviation > 0.3 or right_deviation > 0.3:
                slip_indicators.append(0.3)
        
        # Calculate slip confidence
        self.slip_confidence = float(min(sum(slip_indicators), 1.0))
        
        # Determine if slip is detected
        slip_detected = self.slip_confidence > 0.5
        
        # Handle slip state transitions
        if slip_detected and not self.slip_detected:
            self.slip_start_time = current_time
            self.slip_detected = True
            self.get_logger().warn(f'Wheel slip detected! Confidence: {self.slip_confidence:.2f}')
            
        elif not slip_detected and self.slip_detected:
            if self.slip_start_time is not None:
                slip_duration = (current_time - self.slip_start_time).nanoseconds / 1e9
                if slip_duration > self.slip_recovery_time:
                    self.slip_detected = False
                    self.slip_start_time = None
                    self.get_logger().info('Wheel slip recovery detected')
        
        return self.slip_detected

    def compensate_slip(self, left_vel, right_vel):
        """Apply slip compensation to velocity measurements"""
        if not self.slip_detected:
            return left_vel, right_vel
            
        # Simple slip compensation - reduce confidence in measurements
        compensation_factor = 1.0 - (self.slip_confidence * 0.5)
        
        # Blend with previous measurements during slip
        alpha_slip = 0.8  # More conservative during slip
        compensated_left = alpha_slip * self.filtered_wheel_vel['left'] + (1 - alpha_slip) * left_vel
        compensated_right = alpha_slip * self.filtered_wheel_vel['right'] + (1 - alpha_slip) * right_vel
        
        return compensated_left * compensation_factor, compensated_right * compensation_factor

    def calculate_dynamic_covariance(self, linear_velocity, angular_velocity, acceleration):
        """Updated to remove steering factors"""
        speed = abs(linear_velocity)
        angular_speed = abs(angular_velocity)
        accel_magnitude = abs(acceleration)
        
        pos_var = self.base_position_variance
        pos_var += self.speed_variance_factor * speed
        pos_var += self.acceleration_variance_factor * accel_magnitude
        
        if self.slip_detected:
            slip_factor = 1.0 + (self.slip_confidence * 2.0)
            pos_var *= slip_factor
            
        orient_var = self.base_position_variance + 0.1 * angular_speed
        if self.slip_detected:
            orient_var *= (1.0 + self.slip_confidence)
            
        vel_var = 0.01 + 0.02 * speed
        if self.slip_detected:
            vel_var *= (1.0 + self.slip_confidence * 0.5)
            
        return pos_var, orient_var, vel_var

    def joint_state_callback(self, msg: JointState):
        current_time = self.get_clock().now()
        
        if not self.initialized:
            self.joint_names = list(msg.name)
            required_joints = [self.left_wheel_joint, self.right_wheel_joint]
            missing_joints = [joint for joint in required_joints if joint not in self.joint_names]
            
            if missing_joints:
                self.get_logger().warn(f"Missing joints: {missing_joints}")
                return
            
            try:
                self.last_wheel_pos['left'] = msg.position[self.joint_names.index(self.left_wheel_joint)]
                self.last_wheel_pos['right'] = msg.position[self.joint_names.index(self.right_wheel_joint)]
                self.last_time = current_time
                self.initialized = True
                self.get_logger().info("Differential wheel odometry initialized!")
            except (ValueError, IndexError) as e:
                self.get_logger().error(f"Failed to initialize: {e}")
                return

        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.joint_names = list(msg.name)

        try:
            left_wheel_pos = msg.position[self.joint_names.index(self.left_wheel_joint)]
            right_wheel_pos = msg.position[self.joint_names.index(self.right_wheel_joint)]

            if dt > 0.001: 
                left_wheel_ang_vel = (left_wheel_pos - self.last_wheel_pos['left']) / dt
                right_wheel_ang_vel = (right_wheel_pos - self.last_wheel_pos['right']) / dt
                
                raw_left_vel = left_wheel_ang_vel * self.wheel_radius
                raw_right_vel = right_wheel_ang_vel * self.wheel_radius

                if hasattr(self, 'last_wheel_vel'):
                    max_vel_change = self.max_wheel_acceleration * dt
                    left_vel_change = raw_left_vel - self.last_wheel_vel['left']
                    right_vel_change = raw_right_vel - self.last_wheel_vel['right']
                    
                    if abs(left_vel_change) > max_vel_change:
                        raw_left_vel = self.last_wheel_vel['left'] + math.copysign(max_vel_change, left_vel_change)
                    if abs(right_vel_change) > max_vel_change:
                        raw_right_vel = self.last_wheel_vel['right'] + math.copysign(max_vel_change, right_vel_change)

                if not self.velocity_kf_initialized:
                    self.initialize_velocity_kalman_filter(raw_left_vel, raw_right_vel)
                else:
                    self.predict_velocity_kalman_filter(dt)
                    self.update_velocity_kalman_filter(raw_left_vel, raw_right_vel)
                
                kf_left_vel = self.velocity_state[0]
                kf_right_vel = self.velocity_state[2]
                
                self.detect_wheel_slip(kf_left_vel, kf_right_vel, dt)
                compensated_left_vel, compensated_right_vel = self.compensate_slip(kf_left_vel, kf_right_vel)
                
                self.filtered_wheel_vel['left'] = ((1 - self.velocity_filter_alpha) * self.filtered_wheel_vel['left'] + 
                                                  self.velocity_filter_alpha * compensated_left_vel)
                self.filtered_wheel_vel['right'] = ((1 - self.velocity_filter_alpha) * self.filtered_wheel_vel['right'] + 
                                                   self.velocity_filter_alpha * compensated_right_vel)

                if abs(self.filtered_wheel_vel['left']) < self.deadzone_threshold:
                    self.filtered_wheel_vel['left'] = 0.0
                if abs(self.filtered_wheel_vel['right']) < self.deadzone_threshold:
                    self.filtered_wheel_vel['right'] = 0.0

                self.current_wheel_vel['left'] = self.filtered_wheel_vel['left']
                self.current_wheel_vel['right'] = self.filtered_wheel_vel['right']
                self.last_wheel_vel = {'left': raw_left_vel, 'right': raw_right_vel}

                # Pure Differential Kinematics (No Ackermann Steering Math)
                self.current_linear_velocity = (self.current_wheel_vel['right'] + self.current_wheel_vel['left']) / 2.0
                self.current_angular_velocity = (self.current_wheel_vel['right'] - self.current_wheel_vel['left']) / self.track_width

                self.last_wheel_pos['left'] = left_wheel_pos
                self.last_wheel_pos['right'] = right_wheel_pos
                self.last_time = current_time
                
                self.publish_diagnostics()

        except (ValueError, IndexError) as e:
            pass

    def publish_diagnostics(self):
        """Publish diagnostic information"""
        # Slip detection
        slip_msg = Bool()
        slip_msg.data = self.slip_detected
        self.slip_pub.publish(slip_msg)
        
        # Slip confidence
        confidence_msg = Float64()
        confidence_msg.data = self.slip_confidence
        self.slip_confidence_pub.publish(confidence_msg)
        
        # Wheel velocities
        vel_msg = Vector3()
        vel_msg.x = self.current_wheel_vel['left']
        vel_msg.y = self.current_wheel_vel['right']
        vel_msg.z = (self.current_wheel_vel['left'] + self.current_wheel_vel['right']) / 2.0
        self.wheel_velocities_pub.publish(vel_msg)

    def publish_odometry(self):
        current_time = self.get_clock().now()
        
        if not self.initialized:
            return
        
        if not hasattr(self, 'last_odom_time'):
            self.last_odom_time = current_time
            return
            
        dt = (current_time - self.last_odom_time).nanoseconds / 1e9
        
        # Calculate acceleration for covariance
        if hasattr(self, 'last_linear_velocity'):
            acceleration = (self.current_linear_velocity - self.last_linear_velocity) / max(dt, 0.001)
        else:
            acceleration = 0.0
            
        # Only integrate if time step is reasonable
        if dt > 0.001 and dt < 0.1:
            # Integrate pose
            delta_x = self.current_linear_velocity * math.cos(self.theta) * dt
            delta_y = self.current_linear_velocity * math.sin(self.theta) * dt
            delta_theta = self.current_angular_velocity * dt

            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta
            
            # Normalize theta
            while self.theta > math.pi:
                self.theta -= 2 * math.pi
            while self.theta < -math.pi:
                self.theta += 2 * math.pi
        
        self.last_odom_time = current_time
        self.last_linear_velocity = self.current_linear_velocity

        # Calculate dynamic covariance
        pos_var, orient_var, vel_var = self.calculate_dynamic_covariance(
            self.current_linear_velocity, self.current_angular_velocity, acceleration)

        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Pose
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # Twist
        odom_msg.twist.twist.linear.x = self.current_linear_velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = self.current_angular_velocity
        
        # Enhanced covariance matrices
        odom_msg.pose.covariance = [
            pos_var, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, pos_var, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, orient_var
        ]
        
        odom_msg.twist.covariance = [
            vel_var, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e6, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, vel_var * 2
        ]

        self.odom_pub.publish(odom_msg)
        
        # Debug output (reduced frequency)
        if hasattr(self, 'debug_counter'):
            self.debug_counter += 1
        else:
            self.debug_counter = 0
            
        if self.debug_counter % 100 == 0:  # Every 2 seconds at 50Hz
            slip_status = "SLIP" if self.slip_detected else "OK"
            self.get_logger().info(
                f'Enhanced Odom: x={self.x:.2f}, y={self.y:.2f}, θ={math.degrees(self.theta):.1f}°, '
                f'v={self.current_linear_velocity:.2f}, ω={math.degrees(self.current_angular_velocity):.1f}°/s, '
                f'Status={slip_status}, Conf={self.slip_confidence:.2f}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = EnhancedWheelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
