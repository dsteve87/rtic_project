#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
import tf
from math import sin, cos, pi, atan2, sqrt
from tf.transformations import euler_from_quaternion
import numpy as np

class TurtlebotMultiTrajectory():
    def __init__(self):
        rospy.init_node('turtlebot_multi_trajectory', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'
        self.rate = rospy.Rate(10)  # 10 Hz control loop

        # Robot parameters
        self.L = 160 * 10**-3  # Wheelbase (distance between wheels)
        self.k = 0.20 # SMC gain
        self.dt = 0.1  # Time step (s)

        # Amplitude and frequency for trajectories
        self.A = 2.0  # Infinity/circular amplitude (m)
        self.B = 2.0  # Infinity secondary amplitude (m)
        self.omega = 0.2  # Angular frequency

        # Get transform between odom and base
        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("TF Exception")

    def get_odom(self):
        """Retrieve current position and orientation of the robot."""
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return None, None
        return Point(*trans), rotation[2]

    def sliding_mode_control(self, x, y, theta, x_target, y_target):
        """Compute the SMC control signals."""
        error_x = x_target - x
        error_y = y_target - y
        rho = sqrt(error_x**2 + error_y**2)
        alpha = atan2(error_y, error_x) - theta
        alpha = atan2(sin(alpha), cos(alpha))  # Normalize angle between -pi and pi

        # Sliding surface
        s = rho * sin(alpha)

        # Control commands
        v = self.k * rho * cos(alpha)
        omega = -self.k * s

        return v, omega

    def run(self):
        """Main control loop for generating various trajectories."""
        trajectory_type = input("Enter the trajectory type (infinity, circular, target): ").strip().lower()
        move_cmd = Twist()
        t = 0.0  # Time for trajectory generation

        x_target, y_target = 0.0, 0.0
        target_set = False

        while not rospy.is_shutdown():
            # Retrieve current odometry
            position, theta = self.get_odom()
            if position is None:
                continue

            x = position.x
            y = position.y

            # Calculate target trajectory based on type
            if trajectory_type == "infinity":
                x_target = self.A * cos(self.omega * t)
                y_target = self.B * sin(2 * self.omega * t)

            elif trajectory_type == "circular":
                x_target = self.A * sin(self.omega * t)
                y_target = self.A * cos(self.omega * t)

            elif trajectory_type == "target":
                if not target_set:
                    x_target = float(input("Enter x-coordinate of target: "))
                    y_target = float(input("Enter y-coordinate of target: "))
                    target_set = True

            else:
                rospy.loginfo("Unknown trajectory type.")
                break

            # Calculate control commands
            v, omega = self.sliding_mode_control(x, y, theta, x_target, y_target)

            # Apply control commands
            move_cmd.linear.x = v
            move_cmd.angular.z = omega
            self.cmd_vel.publish(move_cmd)

            # Increment time
            t += self.dt
            self.rate.sleep()

            # Check if reached the target for "target" trajectory
            if trajectory_type == "target" and sqrt((x - x_target)**2 + (y - y_target)**2) < 0.05:
                rospy.loginfo("Reached target position.")
                break

        # Stop the robot after exiting loop
        self.shutdown()

    def shutdown(self):
        """Stop the robot."""
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        TurtlebotMultiTrajectory().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted.")
