import math


def forward_differential(x,y,theta,v_l,v_r,t,l):
    #if moving in straight line vl=vr
    if v_l == v_r:
        x_n = x + v_l * t * math.cos(theta) #integration 3ltool
        y_n = y + v_l * t * math.sin(theta)
        theta_n = theta
    else:
        R=0.5*l*(v_l+v_r)/(v_r-v_l)
         # Compute the angular velocity
        omega = (v_r - v_l) / l
        # Compute the center of the rotation
        ICC_x = x - R * math.sin(theta)
        ICC_y = y + R * math.cos(theta)
        # Update theta
        theta_n = theta + omega * t
        # Use rotation matrix to compute the new position
        x_n = math.cos(omega * t) * (x - ICC_x) - math.sin(omega * t) * (y - ICC_y) + ICC_x
        y_n = math.sin(omega * t) * (x - ICC_x) + math.cos(omega * t) * (y - ICC_y) + ICC_y
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

# Execute commands
for i, (v_l, v_r, t) in enumerate(commands):
    x, y, theta = forward_differential(x, y, theta, v_l, v_r, t, l)
    print(f"After command {i+1}: x = {x:.2f} m, y = {y:.2f} m, θ = {theta:.2f} rad")