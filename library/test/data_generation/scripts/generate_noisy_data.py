"""
    This script generates distorted ellipsoid points with known error from "ideal"
    points on a unit sphere.
"""
import tk_setup
tk_setup.setup_tkinter()

import numpy as np
import matplotlib.pyplot as plt


# References:
#   * https://en.wikipedia.org/wiki/Spherical_coordinate_system
#   * https://www.statisticshowto.com/spherical-coordinates/
#
# If the resources go out of date, any mathematical text will do. Just make sure it has the
# coordinate system defined as:
#   (r, θ, φ), where:
#       * r --> Radial distance
#       * θ --> Polar angle
#       * φ --> Azimuthal angle

# Also make sure the following are satisfied:
#   * r >= 0
#   * 0 <= θ <= π
#   * 0 <= φ <= 2π
#
# In other words, this script uses the Physics convention for spherical coordinates.


# Generate evenly-spaced spherical coordinates for points on the unit sphere.
NUM_POINTS = 50
radius = 1
points_theta = np.linspace(0, np.pi, int(NUM_POINTS / 2))
points_phi = np.linspace(0, 2 * np.pi, NUM_POINTS)

# Convert the spherical point coordinates to cartesian (x, y, z)
theta_grid, phi_grid = np.meshgrid(points_theta, points_phi, indexing='ij')
points_x = radius * np.sin(theta_grid) * np.cos(phi_grid)
points_y = radius * np.sin(theta_grid) * np.sin(phi_grid)
points_z = radius * np.cos(theta_grid)

# Arrange the points in an array of (x, y, z) points
points = np.stack([points_x, points_y, points_z], axis=2)

# TODO: Add known error to cartesian points

# TODO: Add error data to plot to make sure it looks like an ellipsoid
# TODO: Ensure proper proportions are preserved for both shapes on the plot
# Visualize the data to make sure it looks like a sphere
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(points[:, :, 0], points[:, :, 1], points[:, :, 2])
ax.set_box_aspect([1,1,1])
plt.show()
