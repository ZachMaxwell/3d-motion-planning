import matplotlib.pyplot as plt

# Real-world drone position in NED coordinates (North, East, Down)
real_world_north = -0.19428874
real_world_east = -0.87923827

# Offsets for transforming to grid coordinates
north_offset = -316
east_offset = -445

# Calculate grid position after applying offsets
grid_north = real_world_north + 316
grid_east = real_world_east + 445

# Plot settings
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

# =======================
# Plot the Real-World Space
# =======================
ax1.set_title("Real-World Coordinates (NED)")
ax1.set_xlim(-500, 500)  # Adjust based on real-world limits
ax1.set_ylim(-350, 350)  # Adjust based on real-world limits
ax1.set_xlabel("East (m)")
ax1.set_ylabel("North (m)")
ax1.grid(True)

# Plot the real-world bottom-left corner and the current drone position
ax1.scatter(-445, -316, color='blue', label="Bottom-left Corner (-316, -445)")
ax1.scatter(real_world_east, real_world_north, color='red', label="Drone Position (-0.88, -0.19)")

# Add labels to the points
ax1.text(real_world_east, real_world_north, "(NED Position)", color='red', fontsize=10, ha='right')
ax1.text(-445, -316, "(Bottom-left Corner)", color='blue', fontsize=10, ha='right')

# Add a legend
ax1.legend()

# =======================
# Plot the Grid Coordinates
# =======================
ax2.set_title("Grid Coordinates")
ax2.set_xlim(0, 920)  # Adjust grid size limits based on your grid
ax2.set_ylim(0, 920)  # Adjust grid size limits based on your grid
ax2.set_xlabel("East (Grid Units)")
ax2.set_ylabel("North (Grid Units)")
ax2.grid(True)

# Plot the grid origin (0, 0) and the transformed drone position
ax2.scatter(0, 0, color='blue', label="Grid Origin (0, 0)")
ax2.scatter(grid_east, grid_north, color='red', label="Drone Position (315, 444)")

# Add labels to the points
ax2.text(grid_east, grid_north, "(Grid Position)", color='red', fontsize=10, ha='right')
ax2.text(0, 0, "(Grid Origin)", color='blue', fontsize=10, ha='right')

# Add a legend
ax2.legend()

# Show the plot
plt.tight_layout()
plt.show()
