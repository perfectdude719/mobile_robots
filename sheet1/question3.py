import numpy as np
import matplotlib.pyplot as plt

# Robot parameters
r = 0.1  # Wheel radius in meters
l = 0.5  # Half of the distance between the wheels in meters

# Control gains (proportional control)
k_rho = 1     # Control gain for distance to the goal
k_alpha = 7   # Control gain for orientation adjustment towards the goal
k_beta = -2   # Control gain for orientation alignment with the goal

# Proportional control algorithm
def controller(goal_pose, robot_pose, t):
    x, y, theta = robot_pose
    x_goal, y_goal, theta_goal = goal_pose

    # Calculate errors
    dx = x_goal - x
    dy = y_goal - y
    rho = np.sqrt(dx**2 + dy**2)  # Euclidean distance
    flag = 1 if rho <= 0.05 else 0  # Stop if close enough
  
    alpha = np.arctan2(dy, dx) - theta  # Angle to goal
    beta = theta_goal - theta - alpha  # Orientation error

    # Normalize angles to keep them within [-pi, pi]
    alpha = (alpha + np.pi) % (2 * np.pi) - np.pi
    beta = (beta + np.pi) % (2 * np.pi) - np.pi

    # Control laws for linear and angular velocities
    v = k_rho * rho  # Linear velocity proportional to distance
    omega = k_alpha * alpha + k_beta * beta  # Angular velocity proportional to orientation errors

    # Update robot pose based on the control velocities
    delta_x = v * np.cos(theta) * t
    delta_y = v * np.sin(theta) * t
    delta_theta = omega * t

    # Return updated pose and stop flag
    return x + delta_x, y + delta_y, theta + delta_theta, flag


# Simulation parameters
time_steps = np.linspace(0, 50, 500)  # Simulate for 50 seconds with 500 steps
t = 50 / 500  # Time step duration

# Initial and goal poses (start at 0,0 and goal is at 3,4 with orientation pi radians)
robot_pose = (0, 0, np.radians(145))  # Starting pose (x, y, theta in radians)
goal_pose = (3, 4, np.pi)  # Goal pose (x_goal, y_goal, theta_goal)

# Initialize lists to store trajectory data
x_positions = [robot_pose[0]]  # Start x position
y_positions = [robot_pose[1]]  # Start y position
theta_positions = [robot_pose[2]]  # Start orientation

# Trajectory planning
for step in time_steps:
    # Update the robot's pose using the controller
    x, y, theta, flag = controller(goal_pose, robot_pose, t)
    robot_pose = (x, y, theta)
    
    if flag:  # Stop if close enough to goal
        break 
    
    # Store trajectory points
    x_positions.append(x)
    y_positions.append(y)
    theta_positions.append(theta)

# Plot the robot's trajectory and orientation over time
plt.figure(figsize=(12, 6))

# Trajectory plot
plt.subplot(1, 2, 1)
plt.plot(x_positions, y_positions, marker='o', linestyle='-', color='b', label='Trajectory')
plt.plot(goal_pose[0], goal_pose[1], 'rx', label='Goal')  # Mark goal position
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Robot Trajectory')
plt.grid(True)
plt.axis('equal')
plt.legend()

# Orientation plot
plt.subplot(1, 2, 2)
plt.plot(np.linspace(0, len(theta_positions) * t, len(theta_positions)), theta_positions, marker='x', linestyle='-', color='r', label='Theta')
plt.xlabel('Time (s)')
plt.ylabel('Theta (rad)')
plt.title('Robot Orientation Over Time')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()
