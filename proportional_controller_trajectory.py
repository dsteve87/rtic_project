import matplotlib.pyplot as plt
import numpy as np


class WheelMobileRobot:
    def __init__(self, wheel_radius, length, prop_gain):
        self.wheel_radius = wheel_radius
        self.length = length
        self.prop_gain = prop_gain

    def position_error(self, x_target, y_target, theta, x, y):
        error_x = x_target - x
        error_y = y_target - y
        theta_target = np.arctan2(error_y, error_x)
        theta_target = np.arctan2(np.sin(theta_target), np.cos(theta_target))
        error_theta = theta_target - theta
        error_theta = np.arctan2(np.sin(error_theta), np.cos(error_theta))
        return error_x, error_y, error_theta

    def correction_error(self, error_x, error_y, error_theta):
        mse = np.sqrt(error_x**2 + error_y**2)
        theta_dot = self.prop_gain * error_theta
        velocity = self.prop_gain * mse
        return theta_dot, velocity

    def wheel_speeds(self, velocity, theta_dot):
        velocity_vector = np.array([[velocity], [theta_dot]])
        wheel_radius_matrix = np.array(
            [
                [self.wheel_radius / 2, self.wheel_radius / 2],
                [-self.wheel_radius / self.length, self.wheel_radius / self.length],
            ]
        )
        inverse_wheel_radius = np.linalg.inv(wheel_radius_matrix)
        wheel_speeds = np.dot(inverse_wheel_radius, velocity_vector)
        phidotl = wheel_speeds[0, 0]
        phidotr = wheel_speeds[1, 0]
        return phidotl, phidotr

    def diffmodel(self, phidotl, phidotr, theta):
        x_dot = (self.wheel_radius / 2) * (phidotl + phidotr) * np.cos(theta)
        y_dot = (self.wheel_radius / 2) * (phidotl + phidotr) * np.sin(theta)
        theta_dot = (self.wheel_radius / self.length) * (phidotr - phidotl)
        return x_dot, y_dot, theta_dot

    def update_pose(self, x, y, theta, x_dot, y_dot, theta_dot, dt):
        x += x_dot * dt
        y += y_dot * dt
        theta += theta_dot * dt
        theta = np.arctan2(np.sin(theta), np.cos(theta))  # Normalization
        return x, y, theta


def main():
    robot = WheelMobileRobot(wheel_radius=0.5, length=2, prop_gain=1.25)
    x, y, theta = 0.0, 0.0, 0.0
    amplitude = 5
    half_amplitude = amplitude / 2
    omega = 0.2
    positions = [(x, y)]
    distance_errors = []
    orientation_errors = []
    dt = 0.1
    t_final = 60
    time_steps = np.arange(0, t_final, dt)

    print(
        "Input the name of the trajectory you want the robot to describe: infinity, circular, target"
    )
    print("YOU MUST INPUT EITHER: infinity, circular, or target")
    input_trajectory = input("Chosen trajectory:").strip()

    if input_trajectory == "infinity":
        x_theorique = amplitude * np.sin(omega * time_steps)
        y_theorique = half_amplitude * np.sin(2 * omega * time_steps)

        for t in time_steps:
            x_target = amplitude * np.sin(t * omega)
            y_target = half_amplitude * np.sin(2 * t * omega)

            err_x, err_y, err_theta = robot.position_error(x_target, y_target, theta, x, y)
            theta_dot, vb = robot.correction_error(err_x, err_y, err_theta)
            phidotl, phidotr = robot.wheel_speeds(vb, theta_dot)

            x_dot, y_dot, theta_dot = robot.diffmodel(phidotl, phidotr, theta)
            x, y, theta = robot.update_pose(x, y, theta, x_dot, y_dot, theta_dot, dt)

            positions.append((x, y))
            error_distance = np.sqrt(err_x**2 + err_y**2)
            distance_errors.append(error_distance)

            orientation_error = np.arctan2(err_y, err_x) - theta
            orientation_error = np.arctan2(np.sin(orientation_error), np.cos(orientation_error))
            orientation_errors.append(orientation_error)

    elif input_trajectory == "target":
        x_target = float(input("Please input the x-coordinate target: "))
        y_target = float(input("Please input the y-coordinate target: "))

        for t in time_steps:
            err_x, err_y, err_theta = robot.position_error(x_target, y_target, theta, x, y)
            theta_dot, vb = robot.correction_error(err_x, err_y, err_theta)
            phidotl, phidotr = robot.wheel_speeds(vb, theta_dot)

            x_dot, y_dot, theta_dot = robot.diffmodel(phidotl, phidotr, theta)
            x, y, theta = robot.update_pose(x, y, theta, x_dot, y_dot, theta_dot, dt)

            positions.append((x, y))
            error_distance = np.sqrt(err_x**2 + err_y**2)
            distance_errors.append(error_distance)

            orientation_error = np.arctan2(err_y, err_x) - theta
            orientation_error = np.arctan2(np.sin(orientation_error), np.cos(orientation_error))
            orientation_errors.append(orientation_error)

    elif input_trajectory == "circular":
        x_theorique = amplitude * np.sin(omega * time_steps)
        y_theorique = amplitude * np.cos(omega * time_steps)

        for t in time_steps:
            x_target = amplitude * np.sin(t * omega)
            y_target = amplitude * np.cos(t * omega)

            err_x, err_y, err_theta = robot.position_error(x_target, y_target, theta, x, y)
            theta_dot, vb = robot.correction_error(err_x, err_y, err_theta)
            phidotl, phidotr = robot.wheel_speeds(vb, theta_dot)

            x_dot, y_dot, theta_dot = robot.diffmodel(phidotl, phidotr, theta)
            x, y, theta = robot.update_pose(x, y, theta, x_dot, y_dot, theta_dot, dt)

            positions.append((x, y))
            error_distance = np.sqrt(err_x**2 + err_y**2)
            distance_errors.append(error_distance)

            orientation_error = np.arctan2(err_y, err_x) - theta
            orientation_error = np.arctan2(np.sin(orientation_error), np.cos(orientation_error))
            orientation_errors.append(orientation_error)

    else:
        print("The given trajectory name is not known")
        return

    positions = np.array(positions)
    x_vals, y_vals = positions[:, 0], positions[:, 1]

    if len(x_vals) >= 2:
        plt.figure(figsize=(12, 6))

        plt.subplot(1, 2, 1)
        plt.plot(x_vals, y_vals, label="Robot Trajectory", color="blue")
        if input_trajectory != "target":
            plt.plot(x_theorique, y_theorique, label="Theoretical Trajectory", linestyle="--", color="yellow")
        plt.scatter(x_vals[0], y_vals[0], color="red", label="Initial Position")
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        plt.legend()
        plt.title("Robot Trajectory")
        plt.grid()
        plt.axis("equal")

        plt.subplot(1, 2, 2)
        plt.plot(time_steps[:len(distance_errors)], distance_errors, label="Distance Error", color="blue")
        plt.plot(time_steps[:len(orientation_errors)], orientation_errors, label="Orientation Error", color="orange")
        plt.xlabel("Time (s)")
        plt.ylabel("Error")
        plt.legend()
        plt.title("Error Evolution Over Time")
        plt.grid()

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
