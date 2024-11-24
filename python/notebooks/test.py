# ---
# jupyter:
#   jupytext:
#     text_representation:
#       extension: .py
#       format_name: light
#       format_version: '1.5'
#       jupytext_version: 1.16.4
#   kernelspec:
#     display_name: Python 3
#     language: python
#     name: python3
# ---

# %matplotlib widget
from scipy.optimize import minimize
import numpy as np
import matplotlib.pyplot as plt

# +
arm_length = 200
forearm_length = 200

def forward_kinematics(configuration: np.ndarray, L1: float, L2: float) -> np.ndarray:
    assert configuration.shape == (2,)
    theta, phi = configuration

    return np.array([
        L1 * np.cos(theta) * np.cos(phi) - L1 * np.sin(theta) * np.sin(phi) + L2 * np.cos(theta),
        L1 * np.sin(theta) * np.cos(phi) + L1 * np.cos(theta) * np.sin(phi) + L2 * np.sin(theta),
    ])

# Objective Function for Optimization
def objective(joint_angles, target, current_angles, L1, L2):
    end_effector = forward_kinematics(joint_angles, L1, L2)
    position_error = np.linalg.norm(end_effector - target)  # Distance to target
    configuration_error = np.linalg.norm(joint_angles - current_angles)  # Stay close to current config
    return position_error + configuration_error

# IK Solver
def closest_ik(target, current_angles, L1, L2):
    result = minimize(
        objective,
        current_angles,  # Start from current configuration
        args=(target, current_angles, L1, L2),
        method='SLSQP',
        bounds=[(-np.pi / 2, np.pi / 2), (-3 * np.pi / 4, 3 * np.pi / 4)],  # Joint limits
        options={"disp": False}
    )
    print(result)
    return result.x if result.success else None


# +
square_size = 100
square_center = np.array([0, 200])

targets = np.array([
    square_center + np.array([-square_size / 2, square_size / 2]),
    square_center + np.array([square_size / 2, square_size / 2]),
    square_center + np.array([square_size / 2, -square_size / 2]),
    square_center + np.array([-square_size / 2, -square_size / 2]),
])
# -

current_angles = np.array([0, 0])

# +
curr_position = np.array([0, 0, 0, 0])
outputs = []
for target in targets:
    output = closest_ik(target, current_angles, arm_length, forearm_length)
    outputs.append(output)
    current_angles = output

outputs


# +
def plot_configuration(configuration, L1, L2, ax=None):

    if ax is None:
        fig, ax = plt.subplots()

    theta, phi = configuration
    base_pose = np.array([0, 0])

    W_u = np.array([L1 * np.cos(theta), L1 * np.sin(theta), 1])


    W_T_E = np.array([
        [np.cos(theta), -np.sin(theta), W_u[0]],
        [np.sin(theta), np.cos(theta), W_u[1]],
        [0, 0, 1]
    ])

    E_x = np.array([L2 * np.cos(phi), L2 * np.sin(phi), 1])


    W_x = np.dot(W_T_E, E_x)



    # plot the base
    ax.plot(base_pose[0], base_pose[1], 'bo')

    # plot the elbow
    ax.plot(W_u[0], W_u[1], 'go')

    # plot the line between the base and the elbow
    ax.plot([base_pose[0], W_u[0]], [base_pose[1], W_u[1]], 'b-')

    # plot the end_effector
    ax.plot(W_x[0], W_x[1], 'ro')

    # plot the line between the elbow and the end_effector
    ax.plot([W_u[0], W_x[0]], [W_u[1], W_x[1]], 'r-')


    ax.set_aspect('equal')
    ax.set_xlim(-400, 400)
    ax.set_ylim(-20, 400)




fig, axes = plt.subplots(2, 2, figsize=(10, 10))
for ax, output in zip(axes.flatten(), outputs):
    plot_configuration(output, arm_length, forearm_length, ax=ax)

fig.tight_layout()

plt.show()
# -

outputs
