import matplotlib.pyplot as plt

import numpy as np

from scipy.integrate import odeint


#variables

initialState = np.array([0,0,0])
finalState = np.array([10, 10, 0 ])
orientation_gain = 5
velocity_gain = 5
dt = 0.1
radius = 4 
length = 5*radius
x_actuel = initialState[0]
y_actuel = initialState[1]
theta_actuel = initialState[2]

"""paramètres de la trajectoire en forme signe de l'infinie$(())
   x = acos(wt) et y = a/2 x sin(2wt) avec a qui est 
   l'amplitude de cette onde ou trajectoire donc la valeur 
   maximale que peut atteindre x """

A = 5  #Amplitude en x (m) 
B = A/2 #Amplitude en y (m)
omega = 0.2



"""Fonction utilisé pour calculé l'erreur de position"""
def position_error(xd, yd, theta_d, x, y, theta):
    error_x = xd - x 
    error_y = yd - y 
    theta_d = np.arctan2(error_y,error_x)
    theta_d = np.arctan2(np.sin(theta_d), np.cos(theta_d))
    error_theta = theta_d - theta

    return error_x, error_y, error_theta

"""calcule de vb et theta_dot puis multiplication 
    par leur gains respectifs"""
def correction_error(error_x, error_y, error_theta, kv, ktheta):
    mse = np.sqrt(error_x**2 + error_y**2)
    theta_dot = ktheta*(error_theta)
    velocity = kv * mse

    return theta_dot, velocity
"""This function returns the left and right wheel speeds 
    phidotl and phidotr """
def wheel_speeds(velocity, theta_dot, radius, length):
    half_radius = radius/2
    radius_over_length = radius/length
    velocity_vector = np.array([[velocity],[theta_dot]])
    wheel_radius = np.array([[half_radius, half_radius], [-radius_over_length, radius_over_length]])
    inverse_wheel_radius = np.linalg.inv(wheel_radius)
    result = np.matmul(inverse_wheel_radius, velocity_vector)
    phidotl = result[0,0]
    phidotr = result[1,0]
    # print("phidotl = ", phidotl)
    # print("phidotr = ", phidotr)
    return phidotl, phidotr
# wheel_speeds(2,1,radius,length)


"""Calcul de la vitesse en x dx/dt
    vitesse en y dy/dt
    et theta en fonction du temps"""
def diffmodel(theta,time, phidotl, phidotr, radius, length):
    #interpolate the points in the time array with
    #those in the speed array

    x_dot = (radius/2) * (phidotl  + phidotr ) * np.cos(theta)

    y_dot = (radius/2) *  (phidotl  + phidotr) *  np.sin(theta)

    theta_dot = (radius/length)*(phidotr - phidotl)

    v_world = [x_dot, y_dot, theta_dot]
    
    #v_world = np.array(v_world)

    return x_dot, y_dot, theta_dot


def position_actuel(x_dot, y_dot, theta_dot, dt):

    x_dot += x_dot * dt 
    y_dot += y_dot * dt 
    theta_dot += theta_dot * dt
    x_actuel = x_dot
    y_actuel = y_dot
    theta_actuel = theta_dot
    return x_actuel, y_actuel, theta_actuel

x_actuel, y_actuel, theta_actuel =  initialState[0], initialState[0], initialState[0]

positions = [(x_actuel, y_actuel)]
theta_list = []
print('x_actuel, y_actuel, theta_actuel', x_actuel, y_actuel, theta_actuel)

t_final = 60 
time_steps = np.arange(0, t_final , dt)

for t in time_steps:
    x_desire = A * np.sin(t*omega)
    y_desire = B * np.sin(2*t*omega)
    err_x, err_y, err_theta = position_error(x_desire, y_desire, theta_actuel, x_actuel, y_actuel, theta_actuel)
    theta_dot, vb = correction_error(error_x= err_x, error_y= err_y, error_theta= err_theta, kv = velocity_gain, ktheta = orientation_gain )
    phidotl, phidotr = wheel_speeds(velocity= vb, theta_dot= theta_dot, radius=radius, length=length)
    x_dot, y_dot, theta_dot = diffmodel(theta= theta_actuel, time=dt,
                                        phidotl= phidotl, phidotr= phidotr,
                                        radius= radius, length=length)
    
    x_actuel, y_actuel, theta_actuel = position_actuel(x_dot=x_dot, y_dot=y_dot, theta_dot=theta_dot, dt=dt)
    positions.append((x_actuel, y_actuel))

positions = np.array(positions)

x_vals, y_vals = positions[:, 0] , positions[:, 1]


# Affichage de la trajectoire du robot et de la trajectoire théorique
plt.figure(figsize=(12, 6))

# Graphique de la trajectoire
plt.subplot(1, 2, 1)
plt.plot(x_vals, y_vals, label="Trajectoire du robot", color="blue")
# plt.plot(x_theorique, y_theorique, label="Trajectoire théorique", color="green", linestyle="--")
# plt.scatter(x_vals[0], y_vals[0], color='red', label='Position initiale')
# plt.scatter(x_theorique[-1], y_theorique[-1], color='blue', label='Position cible')
plt.xlabel("Position x (m)")
plt.ylabel("Position y (m)")
plt.legend()
plt.title("Trajectoire d'un robot différentiel avec Sliding Mode Control")
plt.grid()
plt.axis("equal")
plt.show()
