"""
    This script generates distorted ellipsoid points with known error from "ideal"
    points on a unit sphere.
"""

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
radius = 1
points_theta = np.linspace(0, np.pi, 50)
points_phi = np.linspace(0, 2 * np.pi, 100)

# Convert the spherical point coordinates to cartesian (x, y, z)
points = []
for i in range(points_theta.size):
    for j in range(points_phi.size):
        x = radius * np.sin(points_theta[i]) * np.cos(points_phi[j])
        y = radius * np.sin(points_theta[i]) * np.sin(points_phi[j])
        z = radius * np.cos(points_theta[i])
        points.append(np.array(points))

points_array = np.array(points)

# TODO: Visualize data and make sure it looks like a sphere

# TODO: Add known error to cartesian points

# TODO: Visualize data and make sure it looks like an ellipsoid
