#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
import tf
from math import sin, cos, pi, atan2, sqrt
from tf.transformations import euler_from_quaternion

class TurtlebotTrajectoryController:
    def __init__(self):
        rospy.init_node('turtlebot_trajectory_controller', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        # Publishers and listeners
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.tf_listener = tf.TransformListener()

        # Set control loop rate
        self.control_rate = rospy.Rate(10)  # 10 Hz

        # Robot parameters (default or fetched from parameter server)
        self.L = rospy.get_param("~wheelbase", 0.16)  # Wheelbase in meters
        self.k = rospy.get_param("~control_gain", 0.20)  # Control gain
        self.dt = rospy.get_param("~time_step", 0.1)  # Time step in seconds

        # Trajectory parameters
        self.A = rospy.get_param("~amplitude_a", 2.0)  # Main amplitude
        self.B = rospy.get_param("~amplitude_b", 2.0)  # Secondary amplitude
        self.omega = rospy.get_param("~angular_frequency", 0.2)  # Frequency

        # Initialize transform frame
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = self.initialize_base_frame()

    def initialize_base_frame(self):
        """Determine the appropriate base frame for the robot."""
        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            rospy.loginfo("Using 'base_footprint' as base frame.")
            return 'base_footprint'
        except tf.Exception:
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                rospy.loginfo("Using 'base_link' as base frame.")
                return 'base_link'
            except tf.Exception:
                rospy.logerr("Failed to find valid base frame. Shutting down.")
                rospy.signal_shutdown("TF Exception")
                return None

    def get_robot_pose(self):
        """Retrieve the robot's current position and orientation."""
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            position = Point(*trans)
            orientation = euler_from_quaternion(rot)[2]
            return position, orientation
        except tf.Exception as e:
            rospy.logwarn(f"TF Exception: {e}")
            return None, None

    def compute_control_signals(self, x, y, theta, x_target, y_target):
        """Compute control commands using a proportional sliding mode controller."""
        error_x = x_target - x
        error_y = y_target - y
        rho = sqrt(error_x ** 2 + error_y ** 2)
        alpha = atan2(error_y, error_x) - theta
        alpha = atan2(sin(alpha), cos(alpha))  # Normalize angle to [-pi, pi]

        # Sliding surface
        s = rho * sin(alpha)

        # Control outputs
        v = self.k * rho
        omega = self.k * s
        return v, omega

    def trajectory_generator(self, trajectory_type, t):
        """Generate the target trajectory."""
        if trajectory_type == "infinity":
            x_target = self.A * cos(self.omega * t)
            y_target = self.B * sin(2 * self.omega * t)
        elif trajectory_type == "circular":
            x_target = self.A * sin(self.omega * t)
            y_target = self.A * cos(self.omega * t)
        else:
            x_target, y_target = None, None
        return x_target, y_target

    def run(self):
        """Main loop to control the robot along the selected trajectory."""
        trajectory_type = input("Enter trajectory type (infinity, circular, target): ").strip().lower()

        if trajectory_type not in {"infinity", "circular", "target"}:
            rospy.logerr("Invalid trajectory type. Exiting.")
            return

        t = 0.0
        move_cmd = Twist()
        target_reached = False

        while not rospy.is_shutdown():
            position, theta = self.get_robot_pose()
            if position is None:
                continue  # Skip loop if unable to fetch pose

            x, y = position.x, position.y

            if trajectory_type in {"infinity", "circular"}:
                x_target, y_target = self.trajectory_generator(trajectory_type, t)
            elif trajectory_type == "target":
                if not target_reached:
                    try:
                        x_target = float(input("Enter x-coordinate of target: "))
                        y_target = float(input("Enter y-coordinate of target: "))
                        target_reached = True
                    except ValueError:
                        rospy.logerr("Invalid input for target coordinates.")
                        break

            # Compute control commands
            v, omega = self.compute_control_signals(x, y, theta, x_target, y_target)

            # Publish commands
            move_cmd.linear.x = v
            move_cmd.angular.z = omega
            self.cmd_vel_pub.publish(move_cmd)

            # Check if target reached (for target trajectory)
            if trajectory_type == "target" and sqrt((x - x_target)**2 + (y - y_target)**2) < 0.05:
                rospy.loginfo("Reached target position.")
                break

            t += self.dt
            self.control_rate.sleep()

        self.shutdown()

    def shutdown(self):
        """Stop the robot."""
        rospy.loginfo("Stopping the robot.")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        TurtlebotTrajectoryController().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
