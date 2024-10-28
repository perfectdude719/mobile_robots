import numpy as np
import matplotlib.pyplot as plt

# Robot parameters
r = 0.1  # Wheel radius in meters
l = 0.5  # Half of the distance between the wheels in meters

# Control gains (proportional control)
k_rho = 0.8  # Increase to drive faster toward the goal
k_alpha = 4.5  # Lower slightly to reduce overshooting
k_beta = -4.0  # Increase for more aggressive orientation correction

# Proportional controller
def controller(goal_pose, robot_pose, t):
    x, y, theta = robot_pose
    x_goal, y_goal, theta_goal = goal_pose

    # Calculate errors
    dx = x_goal - x
    dy = y_goal - y
    rho = np.sqrt(dx**2 + dy**2)  # Euclidean distance
    alpha = np.arctan2(dy, dx) - theta  # Angle to goal
    beta = theta_goal - theta - alpha  # Orientation error

    # Normalize angles to [-pi, pi]
    alpha = (alpha + np.pi) % (2 * np.pi) - np.pi
    beta = (beta + np.pi) % (2 * np.pi) - np.pi

    # Control laws
    v = k_rho * rho
    omega = k_alpha * alpha + k_beta * beta

    # Update pose
    delta_x = v * np.cos(theta) * t
    delta_y = v * np.sin(theta) * t
    delta_theta = omega * t

    # Stop flag checks for both position and orientation accuracy
    flag = 1 if rho <= 0.05 and abs(beta) <= 0.1 else 0

    return x + delta_x, y + delta_y, theta + delta_theta, flag


# Simulation parameters
time_steps = np.linspace(0, 50, 500)  # Simulate for 50 seconds with 500 steps
t = 50 / 500  # Time step duration

# Initial and goal poses
robot_pose = (0, 0,0)  # Starting pose (x, y, theta in radians)
goal_poses = [[np.sin(45),np.sin(45), np.pi/4], [0, 1, np.pi/2], [-np.sin(45),np.sin(45), np.pi*2/3], [-1, 0, np.pi]] # Goal poses

# Initialize lists to store trajectory data
x_positions = [robot_pose[0]]  # Start x position
y_positions = [robot_pose[1]]  # Start y position
theta_positions = [robot_pose[2]]  # Start orientation

# Trajectory planning
for pose in goal_poses:
    for step in time_steps:
        # Update the robot's pose using the controller
        x, y, theta, flag = controller(pose, robot_pose, t)
        robot_pose = (x, y, theta)
    
        if flag:  # Stop if close enough to goal
            break 
    
        # Store trajectory points
        x_positions.append(x)
        y_positions.append(y)
        theta_positions.append(theta)

# Convert theta to degrees for plotting, but only for visualization
theta_positions_deg = np.degrees(theta_positions)

# Plot the robot's trajectory and orientation over time
plt.figure(figsize=(12, 6))

# Trajectory plot
plt.subplot(1, 2, 1)
plt.plot(x_positions, y_positions, marker='o', linestyle='-', color='b', label='Trajectory')
for i, goal in enumerate(goal_poses):
    plt.plot(goal[0], goal[1], 'ro', markersize=10, label=f'Goal {i+1}' if i == 0 else "")  # Label only once in legend
    plt.text(goal[0], goal[1], f"{i+1}",
             fontsize=20, ha='right', color='darkgreen')  # Add goal number and value
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Robot Trajectory with Goal Annotations')
plt.grid(True)
plt.axis('equal')
plt.legend()

# Orientation plot
plt.subplot(1, 2, 2)
plt.plot(np.linspace(0, len(theta_positions_deg) * t, len(theta_positions_deg)), theta_positions_deg, marker='o', linestyle='-', color='r', label='Theta (degrees)')
for i, goal in enumerate(goal_poses):
    goal_time = (len(x_positions) // len(goal_poses)) * t * (i + 1)  # Approximate goal arrival times
    plt.plot(goal_time, np.degrees(goal[2]), 'bx', markersize=10)  # Mark each goal orientation with big red circles
    plt.text(goal_time, np.degrees(goal[2]), f"Goal {i+1}", fontsize=9, ha='right', color='blue')
plt.xlabel('Time (s)')
plt.ylabel('Theta (degrees)')
plt.title('Robot Orientation Over Time')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()
