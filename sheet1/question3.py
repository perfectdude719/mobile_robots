import numpy as np

# Robot parameters
r = 0.1  # Wheel radius
l = 0.5  # Half of the distance between the wheels

# Control gains
k_rho = 1.0
k_alpha = 2.0
k_beta = -1.0

def compute_control(goal, robot_pose):
    x, y, theta = robot_pose
    x_goal, y_goal, theta_goal = goal
    
    # Compute the error in the robot frame
    dx = x_goal - x
    dy = y_goal - y
    rho = np.sqrt(dx**2 + dy**2)
    alpha = np.arctan2(dy, dx) - theta
    beta = theta_goal - theta - alpha
    
    # Control law
    v = k_rho * rho
    omega = k_alpha * alpha + k_beta * beta
    
    return v, omega

def compute_wheel_speeds(v, omega):
    # Compute the wheel angular velocities
    phi_r_dot = (v + l * omega) / r
    phi_l_dot = (v - l * omega) / r
    
    return phi_r_dot, phi_l_dot

# Main loop
goal = (5, 5, 0)  # Example goal pose (x, y, theta)
robot_pose = (0, 0, 0)  # Example current robot pose

v, omega = compute_control(goal, robot_pose)
phi_r_dot, phi_l_dot = compute_wheel_speeds(v, omega)

print(f"Right wheel speed: {phi_r_dot}, Left wheel speed: {phi_l_dot}")
