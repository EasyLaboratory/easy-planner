import matplotlib.pyplot as plt
import numpy as np

# Given values for visualization
# center = np.array([0, 0])


center = np.array([-1, 10])
seed = np.array([0, 1])
seed_init = seed
dp = seed - center
theta0 = np.arctan2(dp[1], dp[0])

desired_dist = 2.5
# theta0 = 0  # Initial angle
resolution = 0.2 # Example resolution
theta_clearance = 0.8 # Example clearance angle

# Calculate angle increment
d_theta = resolution / desired_dist / 2

# Initialize t_l and t_r based on the loop conditions in the code
t_l = theta0 - d_theta
t_r = theta0 + d_theta

# Determine the valid range for t_l
while t_l > theta0 - np.pi / 6:
    p_l = center + desired_dist * np.array([np.cos(t_l), np.sin(t_l)])
    # if not np.isclose(np.linalg.norm(p_l), desired_dist):  # Simulate checkRayValid
    #     t_l += d_theta
    #     break
    t_l -= d_theta

# Determine the valid range for t_r
while t_r < theta0 + np.pi:
    p_r = center + desired_dist * np.array([np.cos(t_r), np.sin(t_r)])
    # if not np.isclose(np.linalg.norm(p_r), desired_dist):  # Simulate checkRayValid
    #     t_r -= d_theta
    #     break
    t_r += d_theta

# Calculate theta_v and visible_p
theta_v = (t_l + t_r) / 2
visible_p = center + desired_dist * np.array([np.cos(theta_v), np.sin(theta_v)])

# Calculate the sector angle theta
theta = (t_r - t_l) / 2
theta_c = min(theta, theta_clearance)

# Determine the updated seed position based on theta0 and the angles
if theta0 - t_l < theta_c:
    seed = center + desired_dist * np.array([np.cos(t_l + theta_c), np.sin(t_l + theta_c)])
elif t_r - theta0 < theta_c:
    seed = center + desired_dist * np.array([np.cos(t_r - theta_c), np.sin(t_r - theta_c)])
else:
    seed = visible_p

# Create figure
fig, ax = plt.subplots()



# Plot center point
ax.plot(center[0], center[1], 'ro', label="Center")


ax.plot(seed_init[0], seed_init[1], 'yo', label="seed Initial")

# Plot visible point
ax.plot(visible_p[0], visible_p[1], 'bo', label="Visible Point (theta_v)")

# Plot the points for t_l and t_r
ax.plot(p_l[0], p_l[1], 'go', label="Point at t_l")
ax.plot(p_r[0], p_r[1], 'mo', label="Point at t_r")

# Plot the updated seed point
ax.plot(seed[0], seed[1], 'co', label="Updated Seed")

# Plot the lines from center to each point
ax.plot([center[0], visible_p[0]], [center[1], visible_p[1]], 'b--', label="Visible Point Line")
ax.plot([center[0], p_l[0]], [center[1], p_l[1]], 'g--', label="Line at t_l")
ax.plot([center[0], p_r[0]], [center[1], p_r[1]], 'm--', label="Line at t_r")
ax.plot([center[0], seed[0]], [center[1], seed[1]], 'c--', label="Line to Updated Seed")
ax.plot([center[0], seed_init[0]], [center[1], seed_init[1]], 'y--', label="Line to init Seed")



# Setting labels and title
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_title('Visible Point and Angles with Seed Update')

# Set equal scaling
ax.axis('equal')

# Show grid
ax.grid(True)

# Show legend
ax.legend()

# Show plot
plt.show()
