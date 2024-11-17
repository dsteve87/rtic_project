import numpy as np
import matplotlib.pyplot as plt

# Paramètres du robot et simulation
dt = 0.1
T = 60  # Durée de la simulation
time_steps = np.arange(0, T, dt)

# Position et orientation initiales
x, y, theta = 0, 0, 0

# Position cible (trajectoire en forme de huit)
target_x = 5 * np.sin(0.1 * time_steps)
target_y = 2.5 * np.sin(0.2 * time_steps)

# Stockage des trajectoires et erreurs
x_trajectory, y_trajectory, theta_trajectory = [], [], []
distance_errors, orientation_errors = [], []

# Boucle de simulation avec contrôle par Sliding Mode Control et perturbations
for i, t in enumerate(time_steps):
    # Erreurs de suivi
    ex = target_x[i] - x
    ey = target_y[i] - y
    distance_error = np.sqrt(ex**2 + ey**2)
    orientation_error = np.arctan2(ey, ex) - theta

    # Enregistrement des erreurs
    distance_errors.append(distance_error)
    orientation_errors.append(orientation_error)

    # Sliding Mode Control
    k_d, k_o = 1.0, 1.0  # Gains du contrôleur
    v = k_d * distance_error * np.cos(orientation_error)
    omega = k_o * orientation_error + np.sign(orientation_error)

    # Mise à jour de la cinématique du robot
    x += v * np.cos(theta) * dt
    y += v * np.sin(theta) * dt
    theta += omega * dt

    # Introduire une perturbation toutes les 10 itérations
    if i % 10 == 0:
        x += np.random.uniform(-0.1, 0.1)  # Perturbation sur x
        y += np.random.uniform(-0.1, 0.1)  # Perturbation sur y
        theta += np.random.uniform(-0.1, 0.1)  # Perturbation sur theta

    # Enregistrement de la trajectoire
    x_trajectory.append(x)
    y_trajectory.append(y)
    theta_trajectory.append(theta)

# Visualisation des résultats
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

# Trajectoire du robot et trajectoire théorique
ax1.plot(target_x, target_y, 'g--', label="Trajectoire théorique")
ax1.plot(x_trajectory, y_trajectory, 'b-', label="Trajectoire du robot")
ax1.plot(x_trajectory[0], y_trajectory[0], 'ro', label="Position initiale")
ax1.plot(x_trajectory[-1], y_trajectory[-1], 'bo', label="Position finale")
ax1.set_xlabel("Position x (m)")
ax1.set_ylabel("Position y (m)")
ax1.set_title("Trajectoire d'un robot différentiel avec Sliding Mode Control")
ax1.legend()

# Évolution des erreurs de suivi au cours du temps
ax2.plot(time_steps, distance_errors, 'b-', label="Erreur de distance")
ax2.plot(time_steps, orientation_errors, 'orange', label="Erreur d'orientation")
ax2.set_xlabel("Temps (s)")
ax2.set_ylabel("Erreur")
ax2.set_title("Évolution des erreurs de suivi au cours du temps")
ax2.legend()

plt.tight_layout()
plt.show()

