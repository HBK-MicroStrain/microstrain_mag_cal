"""
    This script generates distorted points with known error from clean magnetometer data.

    It plots the original data and the data with error for comparison.

    Test fixtures are responsible for applying this error, this script has no effect on them.
    Instead, use this script for exploratory data analysis.
"""
import tk_setup
tk_setup.setup_tkinter()

import numpy as np
import matplotlib.pyplot as plt

from generate_clean_data import generate_clean_magnetometer_data


# Script parameters
# Reference: https://pmc.ncbi.nlm.nih.gov/articles/PMC8401862/#sec2-sensors-21-05288
RADIUS          = 1
NUM_COORDINATES = 50
BIAS            = np.array([2.1, 2.2, 2.3])
SCALE_FACTOR    = [1.1, 2.2, 3.3]
CROSS_COUPLING  = 0.5  # Currently uniform. Refactor for individual cross-coupling.


points = generate_clean_magnetometer_data(num_coordinates=NUM_COORDINATES, radius=RADIUS)

error_matrix = np.full((3, 3), CROSS_COUPLING)   # Fill with uniform cross-coupling error
np.fill_diagonal(error_matrix, SCALE_FACTOR)            # Add scale-factor error

# Einstein summation applies the error to each (x, y, z) point in the tensor. This is a nice
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
points_with_error = np.einsum('ij,...j->...i', error_matrix, points) + BIAS

# Data visualization
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
