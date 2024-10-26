import math
import matplotlib.pyplot as plt

def forward_differential(x, y, theta, v_l, v_r, t, l):
    if v_l == v_r:
        x_n = x + v_l * t * math.cos(theta)
        y_n = y + v_l * t * math.sin(theta)
        theta_n = theta  # No change
    else:
        R = 0.5 * l * (v_l + v_r) / (v_r - v_l)
        omega = (v_r - v_l) / l
        ICC_x = x - R * math.sin(theta)
        ICC_y = y + R * math.cos(theta)

        # Find new positions based on rotation around ICC
        x_n = math.cos(omega * t) * (x - ICC_x) - math.sin(omega * t) * (y - ICC_y) + ICC_x
        y_n = math.sin(omega * t) * (x - ICC_x) + math.cos(omega * t) * (y - ICC_y) + ICC_y
        theta_n = (theta + omega * t) % (2 * math.pi)  # Update orientation
        
    return x_n, y_n, theta_n

# Initial pose
x, y, theta = 1.5, 2.0, math.pi / 2
l = 0.5

# Commands
commands = [
    (0.3, 0.3, 3),     # c1
    (0.1, -0.1, 1),    # c2
    (0.2, 0, 2)        # c3
]

# Lists to store robot's position
x_positions = [x]
y_positions = [y]

# Execute commands
for i, (v_l, v_r, t) in enumerate(commands):
    x, y, theta = forward_differential(x, y, theta, v_l, v_r, t, l)
    x_positions.append(x)
    y_positions.append(y)
    print(f"After command {i+1}: x = {x:.2f} m, y = {y:.2f} m, θ = {theta:.2f} rad")

# Plot the robot's trajectory
plt.plot(x_positions, y_positions, marker='o', linestyle='-', color='b', label='Trajectory')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Robot Trajectory')
plt.grid(True)
plt.legend()
plt.axis('equal')  # Equal scaling for both axes
plt.show()
