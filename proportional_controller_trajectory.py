import matplotlib.pyplot as plt
import numpy as np


class wheelMobileRobot():
    def __init__(self, wheel_radius, length, prop_gain):
        self.wheel_radius = wheel_radius
        self.length = length
        self.prop_gain = prop_gain
        self.amplitude = 5
        self.omega = 0.2



    """Fonction utilisé pour calculé l'erreur de position"""
    def position_error(self,xd, yd, theta_d, x, y, theta):
        error_x = xd - x 
        error_y = yd - y 
        theta_d = np.arctan2(error_y,error_x)
        theta_d = np.arctan2(np.sin(theta_d), np.cos(theta_d))
        error_theta = theta_d - theta
        error_theta = np.arctan2(np.sin(error_theta), np.cos(error_theta))

        return error_x, error_y, error_theta
    

    """calcule de vb et theta_dot puis multiplication 
    par leur gains respectifs"""
    def correction_error(self, error_x, error_y, error_theta):
        mse = np.sqrt(error_x**2 + error_y**2)
        theta_dot = self.prop_gain*(error_theta)
        velocity = self.prop_gain * mse

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
    

    """Calcul de la vitesse en x dx/dt
    vitesse en y dy/dt
    et theta en fonction du temps"""
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
    erreur_distance = []
    erreur_orientation = []
    theta_list = []
    dt = 0.1
    t_final = 60
    time_steps = np.arange(0, t_final, dt)

    print("Input the name of the trajectory you want the robot to describe: infinity, circular, target")
    print("YOU MUST INPUT EITHER: infinity, circular or target")
    input_trajectory = input("Chosen trajectory:")

    if input_trajectory == "infinity":
        
        x_theorique = amplitude * np.sin(omega*time_steps)
        y_theorique = half_amplitude * np .sin(2*omega*time_steps)
        
        
        for t in time_steps:
            x_target = amplitude * np.sin(t*omega)
            y_target = half_amplitude * np.sin(2*t*omega)

             #position error calculation 
            err_x, err_y, err_theta = robot.position_error(x_target, y_target, theta, x, y, theta)

            #velocity and theta_dot 
            theta_dot, vb = robot.correction_error(err_x, err_y, err_theta)
            phidotl, phidotr = robot.wheel_speeds(vb, theta_dot)

            #using the motion equations
            x_dot, y_dot, theta_dot = robot.diffmodel(phidotl,phidotr,theta)

            #actual position and orientation
            x, y, theta = robot.position_actuel(x,y,theta, x_dot, y_dot, theta_dot, dt)

            #saving present position
            positions.append((x, y))

            error_distance = np.sqrt((x - x_target)**2 + (y - y_target)**2)
            erreur_distance.append(error_distance)

            error_orientation = np.arctan2(y_target - y, x_target - x) - theta
            error_orientation = np.arctan2(np.sin(error_orientation), np.cos(error_orientation))
            erreur_orientation.append(error_orientation)

    elif input_trajectory == "target":
        
        x_target = int(input("Please input the coordinates on the x-axis:"))
        y_target = int(input("Please input the coordinates on the y-axis:"))
        theta_target = int(input("please input the coodinates of theta:"))
        
        x_theorique = np.array([x_target])
        y_theorique = np.array([y_target])

        for t in time_steps:
            
            #position error calculation 
            err_x, err_y, err_theta = robot.position_error(x_target, y_target, theta, x, y, theta)

            #velocity and theta_dot 
            theta_dot, vb = robot.correction_error(err_x, err_y, err_theta)
            phidotl, phidotr = robot.wheel_speeds(vb, theta_dot)

            #using the motion equations
            x_dot, y_dot, theta_dot = robot.diffmodel(phidotl,phidotr,theta)

            #actual position and orientation
            x, y, theta = robot.position_actuel(x,y,theta, x_dot, y_dot, theta_dot, dt)

            #saving present position
            positions.append((x, y))

            error_distance = np.sqrt((x - x_target)**2 + (y - y_target)**2)
            erreur_distance.append(error_distance)

            error_orientation = np.arctan2(y_target - y, x_target - x) - theta
            error_orientation = np.arctan2(np.sin(error_orientation), np.cos(error_orientation))
            erreur_orientation.append(error_orientation)

        
    elif input_trajectory == "circular":
        x_theorique = amplitude * np.sin(omega * time_steps)
        y_theorique = amplitude * np.cos(omega * time_steps)
        
        for t in time_steps:
            x_target = amplitude *  np.sin(t*omega)
            y_target = amplitude * np.cos(t*omega)
            #position error calculation 
            err_x, err_y, err_theta = robot.position_error(x_target, y_target, theta, x, y, theta)

            #velocity and theta_dot 
            theta_dot, vb = robot.correction_error(err_x, err_y, err_theta)
            phidotl, phidotr = robot.wheel_speeds(vb, theta_dot)

            #using the motion equations
            x_dot, y_dot, theta_dot = robot.diffmodel(phidotl,phidotr,theta)

            #actual position and orientation
            x, y, theta = robot.position_actuel(x,y,theta, x_dot, y_dot, theta_dot, dt)

            #saving present position
            positions.append((x, y))

            error_distance = np.sqrt((x - x_target)**2 + (y - y_target)**2)
            erreur_distance.append(error_distance)

            error_orientation = np.arctan2(y_target - y, x_target - x) - theta
            error_orientation = np.arctan2(np.sin(error_orientation), np.cos(error_orientation))
            erreur_orientation.append(error_orientation)
    
    else:
        print("The given trajectory name is not known")
    positions = np.array(positions)
    x_vals, y_vals = positions[:, 0], positions[:, 1]
        # Affichage de la trajectoire du robot et de la trajectoire théorique
    print(x_vals.shape[0])
    if x_vals.shape[0] >= 2:
        plt.figure(figsize=(12, 6))

        # Graphique de la trajectoire
        plt.subplot(1, 2, 1)
        plt.plot(x_vals, y_vals, label="Trajectoire du robot", color="blue")
        plt.plot(x_theorique, y_theorique, label="Trajectoire théorique", color="yellow", linestyle="--")
        plt.scatter(x_vals[0], y_vals[0], color='red', label='Position initiale')
        plt.scatter(x_theorique[-1], y_theorique[-1], color='blue', label='Position cible')
        plt.xlabel("Position x (m)")
        plt.ylabel("Position y (m)")
        plt.legend()
        plt.title("Trajectoire d'un robot différentiel avec Sliding Mode Control")
        plt.grid()
        plt.axis("equal")


        # Graphique des erreurs de suivi
        plt.subplot(1, 2, 2)
        plt.plot(time_steps, erreur_distance, label="Erreur de distance", color="blue")
        plt.plot(time_steps, erreur_orientation, label="Erreur d'orientation", color="orange")
        plt.xlabel("Temps (s)")
        plt.ylabel("Erreur")
        plt.legend()
        plt.title("Évolution des erreurs de suivi au cours du temps")
        plt.grid()

        plt.tight_layout()
        plt.show()



if __name__ == "__main__":
    main()