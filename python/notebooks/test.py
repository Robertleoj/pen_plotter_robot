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

# # %matplotlib widget
from scipy.optimize import minimize
import numpy as np
import matplotlib.pyplot as plt

# +
arm_length = 196.15
forearm_length = 186.321 + 6.1

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
    # print(result)
    return result.x if result.success else None


# +
def get_square():
    square_size = 100
    square_center = np.array([0, 300])

    targets = np.array([
        square_center + np.array([-square_size / 2, square_size / 2]),
        square_center + np.array([square_size / 2, square_size / 2]),
        square_center + np.array([square_size / 2, -square_size / 2]),
        square_center + np.array([-square_size / 2, -square_size / 2]),
    ])
    return targets

def get_circle():
    circle_center = np.array([0, 250])
    circle_radius = 50
    circle_points = np.array([circle_center + circle_radius * np.array([np.cos(theta), np.sin(theta)]) for theta in np.linspace(0, 2 * np.pi, 20)])
    return circle_points


def center_points(points, center, size):
    x_range = np.max(points[:, 0]) - np.min(points[:, 0])
    y_range = np.max(points[:, 1]) - np.min(points[:, 1])

    scale = max(x_range, y_range) / size

    points = (points - np.mean(points, axis=0)) / scale + center

    return points

    

def get_heart(num_points: int = 1000):
    # Scale and position parameters
    scale = 50
    center = np.array([0, 250])
    
    # Generate points for the heart curve
    t = np.linspace(0, 2*np.pi, num_points)
    x = 16 * np.sin(t)**3
    y = 13 * np.cos(t) - 5 * np.cos(2*t) - 2 * np.cos(3*t) - np.cos(4*t)
    
    # Scale and translate the points
    points = np.column_stack([x, y])
    points = scale * points
    points = points + center

    return points




# -

targets = get_heart(100)
targets = center_points(targets, np.array([0, 250]), 100)

# +
# generate a circle of points
fig, ax = plt.subplots(figsize=(10, 10))
ax.scatter(targets[:, 0], targets[:, 1], c='green')
ax.axis('equal')
plt.show()


# -

current_angles = np.array([0, 0])

# +
curr_position = np.array([0, 0, 0, 0])
outputs = []
for target in targets:
    output = closest_ik(target, current_angles, arm_length, forearm_length)
    outputs.append(output)
    current_angles = output

outputs = np.array(outputs)
outputs.shape


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

# +
import matplotlib.animation as animation

def animate_configurations(outputs, L1, L2):
    fig, ax = plt.subplots(figsize=(8, 8))
    
    def init():
        ax.set_xlim(-400, 400)
        ax.set_ylim(-20, 400)
        ax.set_aspect('equal')
        return []
    
    def animate(frame):
        ax.clear()
        ax.set_xlim(-400, 400)
        ax.set_ylim(-20, 400)
        ax.set_aspect('equal')
        
        # Your existing plotting logic
        plot_configuration(outputs[frame], L1, L2, ax=ax)
        ax.scatter(targets[:, 0], targets[:, 1], c='red')
        return []
    
    anim = animation.FuncAnimation(
        fig, 
        animate,
        init_func=init,
        frames=len(outputs),
        interval=100,  # Time between frames in milliseconds
        blit=True
    )
    
    plt.close()  # Prevents duplicate display in notebooks
    return anim

num_plot_points = 30
every_n_points = int(len(outputs) / num_plot_points)
# Create and display the animation
anim = animate_configurations(outputs[::every_n_points], arm_length, forearm_length)
from IPython.display import HTML
HTML(anim.to_jshtml())
# -

outputs

# +
# ... existing imports ...
import serial
import struct
import time

ser = serial.Serial(
    port='/dev/ttyUSB1',  # Adjust port as needed
    baudrate=115200,
    timeout=1
)

def send_target(angles):
    """Send a pair of angles over serial.
    Formats as binary: <theta><phi> (2 floats = 8 bytes)
    """
    # Pack two float32s into binary data
    ser.write(f"{angles[0]}\n{angles[1]}\n".encode('utf-8'))
    # Small delay to prevent overwhelming the serial buffer
    time.sleep(0.01)

# Send all targets
try:
    for target in outputs:
        send_target(target)
    ser.write(b'\n')
    print(f"Sent {len(outputs)} targets")
    while True:
        print(ser.readline())

finally:
    ser.close()

# read responses forever


