"""
    This script generates distorted points with known error from clean points on a unit sphere.

    It plots the original data and the data with error for comparison.
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

# Group the cartesian coordinates as a rank-3 tensor. The structure is as follows:
#   * Three "sub-grids" (x, y, or z)
#   * Each grid contains that grid's coordinate (x, y, or z) for each of the original theta-phi pairs.
#
# This allows us to query a point (x, y, z) for each pair by indexing [i, j, c],
#   where i = theta, j = phi, c = coordinate (x, y, or z).
#
points = np.stack([points_x, points_y, points_z], axis=2)

# Add known error (reference these when writing automated tests).
# Reference: https://pmc.ncbi.nlm.nih.gov/articles/PMC8401862/#sec2-sensors-21-05288
bias = np.array([2.1, 2.2, 2.3])
error_matrix = np.full((3, 3), 0.5)   # Fill with uniform cross-coupling error
np.fill_diagonal(error_matrix, [1.1, 2.2, 3.3])  # Add scale-factor error

# Einstein summation is applying the error to each (x, y, z) point in the tensor. This is a nice
# trick to apply the error to each point without having to flatten the tensor to a Nx3 matrix
# and transpose it.
#
# Reference: https://en.wikipedia.org/wiki/Einstein_notation#Matrix_multiplication
#
# Index legend:
#   *   ij ---> i = row, j = col in error matrix
#   * ...j ---> j = coordinate grid (x: j = 1, y: j = 2, z: j = 3)
#   * ...i ---> Result has shape (..., i), sum over j
#
# Adding the bias takes advantage of Numpy's broadcasting system:
# https://numpy.org/doc/stable/user/basics.broadcasting.html
#
points_with_error = np.einsum('ij,...j->...i', error_matrix, points) + bias

# Write the points to output file so they can be extracted to

# Visualize the data. The original data should look like a sphere, while the data with error should
# look like an ellipsoid.
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(
    points[:, :, 0], points[:, :, 1], points[:, :, 2],
    label='Original',
    color='green')
ax.scatter(
    points_with_error[:, :, 0], points_with_error[:, :, 1], points_with_error[:, :, 2],
    label='Error',
    color='red')
ax.set_aspect('equal')
ax.legend()
plt.show()
