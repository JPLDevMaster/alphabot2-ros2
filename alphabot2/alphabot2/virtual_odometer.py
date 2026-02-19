import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from alphabot2_interfaces.msg import Obstacle
from tf2_ros import TransformBroadcaster

VIRTUAL_ODOMETRY_TOPIC = "virtual_odometry"
CMD_VEL_TOPIC = "cmd_vel"
OBSTACLES_TOPIC = "obstacles"
SPIN_TIMER_PERIOD_SEC = 0.025   # Timer callback period (40 Hz)

def yaw_to_quaternion(yaw):
    """
    Helper function to convert a yaw angle (in radians) to a quaternion.
    ROS2 requires orientations to be in quaternion format (x, y, z, w).
    For a 2D robot, roll and pitch are 0.
    """
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qx, qy, qz, qw

class VirtualOdometer(Node):
    """
    ROS2 node that emulates a virtual odometer.
    It integrates 'cmd_vel' over time to estimate the robot's pose (x, y, theta)
    and publishes standard nav_msgs/Odometry and tf2 transforms.
    """
    def __init__(self):
        super().__init__("virtual_odometer")

        self.get_logger().info("Virtual Odometer Node init ...")

        # Sync the emergency stop parameter with the motion_driver.
        self.declare_parameter('use_obstacle_avoidance_emergency_stop', False)

        # Topics publisher and subscribers.
        self.cmd_vel_sub = self.create_subscription(Twist, CMD_VEL_TOPIC, self.cmd_vel_sub_callback, 10)
        self.obstacles_sub = self.create_subscription(Obstacle, OBSTACLES_TOPIC, self.obstacles_sub_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, VIRTUAL_ODOMETRY_TOPIC, 10)
        
        # Transform broadcaster to link 'odom' frame to 'base_link' frame.
        self.tf_broadcaster = TransformBroadcaster(self)

        # Internal Robot State (Pose).
        self.x = 0.0      # meters.
        self.y = 0.0      # meters.
        self.theta = 0.0  # radians (yaw).

        # Current velocity.
        self.linear_v = 0.0
        self.angular_w = 0.0
        self.obstacle_detected = False

        # Time tracking for integration.
        self.last_time = self.get_clock().now()

        # Create the timer (Integration Loop).
        self.timer = self.create_timer(SPIN_TIMER_PERIOD_SEC, self.integration_loop_callback)

        self.get_logger().info("Virtual Odometer Node init complete.")

    def cmd_vel_sub_callback(self, twist_msg):
        """
        Function called when there is a new Twist message on the "cmd_vel" topic.
        :param twist_msg: received Twist message.
        """

        # Set linear and angular copying them from the "cmd_vel" message.
        self.linear_v = twist_msg.linear.x
        self.angular_w = twist_msg.angular.z

    def obstacles_sub_callback(self, obstacle_msg):
        """
        Function called when there is a new Obstacle message on the "obstacles" topic.
        :param obstacle_msg: received Obstacle message.
        """

        # Update the internal status.
        self.obstacle_detected = obstacle_msg.right_obstacle or obstacle_msg.left_obstacle

    def integration_loop_callback(self):
        """
        This is the core mathematical loop. It calculates how much time has passed
        and updates the robot's X, Y, and Theta based on its current velocity.
        """

        # Get the current time.
        current_time = self.get_clock().now()
        
        # Calculate delta time (dt) in seconds.
        dt = (current_time - self.last_time).nanoseconds / 1e9

        # Get current parameter value for emergency stop.
        use_emergency_stop = self.get_parameter('use_obstacle_avoidance_emergency_stop').get_parameter_value().bool_value

        # If the motion_driver stopped the wheels due to an obstacle,
        # our virtual odometer MUST also know the velocity is 0, otherwise the virtual position will drift.
        actual_linear_v = self.linear_v
        if use_emergency_stop and self.obstacle_detected and actual_linear_v > 0:
            actual_linear_v = 0.0

        # Calculate the change in position and heading over this time slice (dt).
        delta_x = actual_linear_v * math.cos(self.theta) * dt
        delta_y = actual_linear_v * math.sin(self.theta) * dt
        delta_theta = self.angular_w * dt

        # Add the changes to the global state.
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Convert the new yaw angle to a quaternion.
        qx, qy, qz, qw = yaw_to_quaternion(self.theta)

        # Publish the transform.
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(t)

        # Publish the odometry message.
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set the position.
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Set the velocity.
        odom_msg.twist.twist.linear.x = actual_linear_v
        odom_msg.twist.twist.angular.z = self.angular_w

        self.odom_pub.publish(odom_msg)

        # Update last_time for the next loop.
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    virtual_odometer = VirtualOdometer()
    rclpy.spin(virtual_odometer)
    virtual_odometer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()