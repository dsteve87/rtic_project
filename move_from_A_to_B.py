import numpy as np
import matplotlib.pyplot as plt

# Paramètres du robot
L = 0.5   # Distance entre les roues (m)
r = 0.1   # Rayon des roues (m)
dt = 0.1  # Intervalle de temps pour la simulation (s)

# Paramètres du contrôleur de glissement
k = 5.0   # Gain du contrôleur de glissement

# Fonction de mise à jour de la position et de l'orientation du robot
def update_pose(x, y, theta, v_L, v_R, L, dt):
    # Calcul de la vitesse linéaire et angulaire du robot
    v = (v_R + v_L) / 2
    omega = (v_R - v_L) / L
    
    # Mise à jour de la position et de l'orientation
    x += v * np.cos(theta) * dt
    y += v * np.sin(theta) * dt
    theta += omega * dt
    
    return x, y, theta

# Fonction de calcul du contrôleur de glissement (SMC)
def sliding_mode_control(x, y, theta, x_target, y_target, k):
    # Calcul de l'erreur de position
    error_x = x_target - x
    error_y = y_target - y
    
    # Erreur de distance et angle à la cible
    rho = np.sqrt(error_x**2 + error_y**2)
    alpha = np.arctan2(error_y, error_x) - theta
    alpha = np.arctan2(np.sin(alpha), np.cos(alpha))  # Contrainte sur l'angle entre -pi et pi
    
    # Surface de glissement (sliding surface)
    s = rho * np.sin(alpha)
    
    # Commande en vitesse linéaire et angulaire pour réduire l'erreur
    v = k * rho * np.cos(alpha)
    omega = -k * s
    
    # Calcul des vitesses des roues gauche et droite
    v_L = v - (omega * L / 2)
    v_R = v + (omega * L / 2)
    
    return v_L, v_R

# Conditions initiales du robot
x, y, theta = 0.0, 0.0, 0.0  # Position initiale (x, y) et orientation (theta)

# Définition de la position cible
x_target, y_target = 5.0, 5.0  # Position cible en (x, y)

# Initialisation des listes pour stocker les positions et les erreurs
positions = [(x, y)]           # Stockage des positions pour affichage
distance_errors = []            # Erreurs de distance
orientation_errors = []         # Erreurs d'orientation

# Simulation pour une durée de 20 secondes
t_final = 20
for _ in np.arange(0, t_final, dt):
    # Calcul des vitesses de roues avec le contrôleur de glissement
    v_L, v_R = sliding_mode_control(x, y, theta, x_target, y_target, k)
    
    # Mise à jour de la position et de l'orientation du robot
    x, y, theta = update_pose(x, y, theta, v_L, v_R, L, dt)
    
    # Calcul des erreurs
    error_distance = np.sqrt((x - x_target)**2 + (y - y_target)**2)
    error_orientation = np.arctan2(y_target - y, x_target - x) - theta
    error_orientation = np.arctan2(np.sin(error_orientation), np.cos(error_orientation))  # Contraindre entre -pi et pi

    # Enregistrement des erreurs
    distance_errors.append(error_distance)
    orientation_errors.append(error_orientation)
    
    # Enregistrement de la position pour l'affichage
    positions.append((x, y))
    
    # Arrêt si le robot atteint la cible
    if error_distance < 0.1:
        break

# Extraction des positions pour l'affichage
positions = np.array(positions)
x_vals, y_vals = positions[:, 0], positions[:, 1]

# Affichage de la trajectoire du robot
plt.figure(figsize=(12, 6))

# Sous-figure 1: Trajectoire du robot
plt.subplot(1, 2, 1)
plt.plot(x_vals, y_vals, label="Trajectoire du robot")
plt.scatter(x_vals[0], y_vals[0], color='red', label='Position initiale')
plt.scatter(x_target, y_target, color='blue', label='Position cible')
plt.xlabel("Position x (m)")
plt.ylabel("Position y (m)")
plt.legend()
plt.title("Trajectoire d'un robot différentiel avec Sliding Mode Control")
plt.grid()
plt.axis("equal")

# Sous-figure 2: Évolution des erreurs
plt.subplot(1, 2, 2)
plt.plot(distance_errors, label="Erreur de distance")
plt.plot(orientation_errors, label="Erreur d'orientation")
plt.xlabel("Itérations")
plt.ylabel("Erreur")
plt.legend()
plt.title("Évolution des erreurs de suivi au cours du temps")
plt.grid()

plt.tight_layout()
plt.show()

