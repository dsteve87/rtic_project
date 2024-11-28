import matplotlib.pyplot as plt
import numpy as np


class wheelMobileRobot():
    def __init__(self, wheel_radius, length, prop_gain):
        self.wheel_radius = wheel_radius
        self.length = length
        self.prop_gain = prop_gain
        self.amplitude = 5
        self.omega = 0.2



    """fonction used to compute erro"""
    def position_error(self,xd, yd, theta_d, x, y, theta):
        error_x = xd - x 
        error_y = yd - y 
        theta_d = np.arctan2(error_y,error_x)
        theta_d = np.arctan2(np.sin(theta_d), np.cos(theta_d))
        error_theta = theta_d - theta
        error_theta = np.arctan2(np.sin(error_theta), np.cos(error_theta))

        return error_x, error_y, error_theta
    

    """compute vb and theta_dot then apply the gain"""
    def sliding_mode(self, error_x, error_y, error_theta):
        
        rho = np.sqrt(error_x**2 + error_y**2)
        
        #sliding surface

        s = rho * np.sin(error_theta)

        #linear and angular velocity commands 
        theta_dot = -1 * self.prop_gain * s
        velocity = self.prop_gain * rho * np.cos(error_theta)

        return theta_dot, velocity
    

    """This function returns the left and right wheel speeds 
    phidotl and phidotr """
    def wheel_speeds(self, velocity, theta_dot):
        velocity_vector = np.array([[velocity],[theta_dot]])
        wheel_radius_matrix = np.array([[self.wheel_radius/2, self.wheel_radius/2], [-self.wheel_radius/self.length, self.wheel_radius/self.length]])
        inverse_wheel_radius = np.linalg.inv(wheel_radius_matrix)
        wheel_speeds = np.dot(inverse_wheel_radius, velocity_vector)
        phidotl = wheel_speeds[0,0]
        phidotr = wheel_speeds[1,0]

        return phidotl, phidotr
    

    """Compute theta, the velocity on x (x dx/dt) and y (y dy/dt) direction with respect to the time"""
    def diffmodel(self, phidotl, phidotr, theta):
        #interpolate the points in the time array with
        #those in the speed array

        x_dot = (self.wheel_radius/2) * (phidotl  + phidotr ) * np.cos(theta)

        y_dot = (self.wheel_radius/2) *  (phidotl  + phidotr) *  np.sin(theta)

        theta_dot = (self.wheel_radius/self.length)*(phidotr - phidotl)

        v_world = [x_dot, y_dot, theta_dot]

        #v_world = np.array(v_world)

        return x_dot, y_dot, theta_dot
    

    def position_actuel(self, x, y, theta, x_dot, y_dot, theta_dot, dt):
        x += x_dot * dt 
    
        y += y_dot * dt 
    
        theta += theta_dot * dt
    
        theta = np.arctan2(np.sin(theta), np.cos(theta)) #normalisation
    
        return x, y, theta



def main():
    #class object
    robot = wheelMobileRobot(wheel_radius= 0.5 , length= 2, prop_gain=1.25)
    #variables
    x, y, theta = 0.0, 0.0, 0.0
    amplitude = 5
    half_amplitude = amplitude/2
    omega = 0.2
    positions = [(x,y)]
    erro_distance_list = []
    error_orientation_list = []
    theta_list = []
    dt = 0.1
    t_final = 60
    time_steps = np.arange(0, t_final, dt)

    print("Input the name of the trajectory you want the robot to describe: infinity, circular, target")
    print("YOU MUST INPUT EITHER: infinity, circular or target")
    input_trajectory = input("Chosen trajectory:")

    if input_trajectory == "infinity":
        
        x_theorical = amplitude * np.sin(omega*time_steps)
        y_theorical = half_amplitude * np .sin(2*omega*time_steps)
        
        
        for t in time_steps:
            x_target = amplitude * np.sin(t*omega)
            y_target = half_amplitude * np.sin(2*t*omega)

             #position error calculation 
            err_x, err_y, err_theta = robot.position_error(x_target, y_target, theta, x, y, theta)

            #velocity and theta_dot 
            theta_dot, vb = robot.sliding_mode(err_x, err_y, err_theta)
            phidotl, phidotr = robot.wheel_speeds(vb, theta_dot)

            #using the motion equations
            x_dot, y_dot, theta_dot = robot.diffmodel(phidotl,phidotr,theta)

            #actual position and orientation
            x, y, theta = robot.position_actuel(x,y,theta, x_dot, y_dot, theta_dot, dt)

            #saving present position
            positions.append((x, y))

            error_distance = np.sqrt((x - x_target)**2 + (y - y_target)**2)
            erro_distance_list.append(error_distance)

            error_orientation = np.arctan2(y_target - y, x_target - x) - theta
            error_orientation = np.arctan2(np.sin(error_orientation), np.cos(error_orientation))
            error_orientation_list.append(error_orientation)

    elif input_trajectory == "target":
        
        x_target = int(input("Please input the coordinates on the x-axis:"))
        y_target = int(input("Please input the coordinates on the y-axis:"))
        theta_target = int(input("please input the coodinates of theta:"))
        
        x_theorical = np.array([x_target])
        y_theorical = np.array([y_target])

        for t in time_steps:
            
            #position error calculation 
            err_x, err_y, err_theta = robot.position_error(x_target, y_target, theta, x, y, theta)

            #velocity and theta_dot 
            theta_dot, vb = robot.sliding_mode(err_x, err_y, err_theta)
            phidotl, phidotr = robot.wheel_speeds(vb, theta_dot)

            #using the motion equations
            x_dot, y_dot, theta_dot = robot.diffmodel(phidotl,phidotr,theta)

            #actual position and orientation
            x, y, theta = robot.position_actuel(x,y,theta, x_dot, y_dot, theta_dot, dt)

            #saving present position
            positions.append((x, y))

            error_distance = np.sqrt((x - x_target)**2 + (y - y_target)**2)
            erro_distance_list.append(error_distance)

            error_orientation = np.arctan2(y_target - y, x_target - x) - theta
            error_orientation = np.arctan2(np.sin(error_orientation), np.cos(error_orientation))
            error_orientation_list.append(error_orientation)

        
    elif input_trajectory == "circular":
        x_theorical = amplitude * np.sin(omega * time_steps)
        y_theorical = amplitude * np.cos(omega * time_steps)
        
        for t in time_steps:
            x_target = amplitude *  np.sin(t*omega)
            y_target = amplitude * np.cos(t*omega)
            #position error calculation 
            err_x, err_y, err_theta = robot.position_error(x_target, y_target, theta, x, y, theta)

            #velocity and theta_dot 
            theta_dot, vb = robot.sliding_mode(err_x, err_y, err_theta)
            phidotl, phidotr = robot.wheel_speeds(vb, theta_dot)

            #using the motion equations
            x_dot, y_dot, theta_dot = robot.diffmodel(phidotl,phidotr,theta)

            #actual position and orientation
            x, y, theta = robot.position_actuel(x,y,theta, x_dot, y_dot, theta_dot, dt)

            #saving present position
            positions.append((x, y))

            error_distance = np.sqrt((x - x_target)**2 + (y - y_target)**2)
            erro_distance_list.append(error_distance)

            error_orientation = np.arctan2(y_target - y, x_target - x) - theta
            error_orientation = np.arctan2(np.sin(error_orientation), np.cos(error_orientation))
            error_orientation_list.append(error_orientation)
    
    else:
        print("The given trajectory name is not known")
    positions = np.array(positions)
    x_vals, y_vals = positions[:, 0], positions[:, 1]
        # Display of robot trajectory and theorical trajectory 
    #print(x_vals.shape[0])
    
    #this condition is used because we want to 
    #represent only the known trajectories
    if x_vals.shape[0] >= 2:
        plt.figure(figsize=(12, 6))

        # Graph of trajectory
        plt.subplot(1, 2, 1)
        plt.plot(x_vals, y_vals, label="robot trajectory", color="blue")
        plt.plot(x_theorical, y_theorical, label="theorical Trajectory", color="yellow", linestyle="--")
        plt.scatter(x_vals[0], y_vals[0], color='red', label='initial Position')
        plt.scatter(x_theorical[-1], y_theorical[-1], color='blue', label='target Position')
        plt.xlabel("Position x (m)")
        plt.ylabel("Position y (m)")
        plt.legend()
        plt.title("Trajectory of differential drive robot with Sliding Mode Control")
        plt.grid()
        plt.axis("equal")


        # Graph of errors
        plt.subplot(1, 2, 2)
        plt.plot(time_steps, erro_distance_list, label="distance error", color="blue")
        plt.plot(time_steps, error_orientation_list, label="orientation error", color="orange")
        plt.xlabel("Time (s)")
        plt.ylabel("Error")
        plt.legend()
        plt.title("evolution of errors with respect to the time")
        plt.grid()

        plt.tight_layout()
        plt.show()



if __name__ == "__main__":
    main()
