"""
    This module is used to generate clean magnetometer data (for automated testing).

    The data can be used by test fixtures to apply intentional error for testing calibration. It
    can also be used in further scripts for visualization and data analysis.

    If run as a script, it writes the data to a C++ header file as an object that can be used by
    test fixtures.
"""
import pathlib

import numpy as np


def generate_clean_magnetometer_data(num_coordinates, radius=1):
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
    # In other words, this uses the Physics convention for spherical coordinates.

    points_theta = np.linspace(0, np.pi, int(num_coordinates / 2))
    points_phi = np.linspace(0, 2 * np.pi, num_coordinates)

    theta_grid, phi_grid = np.meshgrid(points_theta, points_phi, indexing='ij')
    points_x = radius * np.sin(theta_grid) * np.cos(phi_grid)
    points_y = radius * np.sin(theta_grid) * np.sin(phi_grid)
    points_z = radius * np.cos(theta_grid)

    # Group the cartesian coordinates as a rank-3 tensor. The structure is as follows:
    #   * Three "sub-grids" (x, y, or z)
    #   * Each grid contains that grid's coordinate (x, y, or z) for each of the original theta-phi
    #     pairs.
    #
    # This allows us to query a point (x, y, z) for each pair by indexing [i, j, c],
    #   where i = theta, j = phi, c = coordinate (x, y, or z).
    #
    return np.stack([points_x, points_y, points_z], axis=2)

def save_data_as_cpp_header(points_flattened, filepath, radius=1):
    with open(filepath, "w") as file:
        file.write(f"/* \n")
        file.write(f" * Auto-generated test data file - DON'T EDIT MANUALLY!\n")
        file.write(f" * Generated from spherical coordinates on sphere with radius: {radius}\n")
        file.write(f" */\n")
        file.write(f"#pragma once\n")
        file.write(f"\n")
        file.write(f"#include <array>\n")
        file.write(f"\n")
        file.write(f"#include <Eigen/Dense>\n")
        file.write(f"\n")
        file.write(f"namespace fixture\n")
        file.write(f"{{\n")

        rows = points_flattened.shape[0]
        cols = points_flattened.shape[1]

        file.write(f"    static constexpr std::array raw_data {{\n")
        for i, row in enumerate(points_flattened):
            comma = "," if i < rows - 1 else ""
            x, y, z = row
            # IEEE double holds ~15 digits of precision.
            # Reference: https://en.wikipedia.org/wiki/Double-precision_floating-point_format#IEEE_754_double-precision_binary_floating-point_format:_binary64
            # Use width specifier to accommodate sign + decimal point + digits + exponent
            file.write(f"        {x:22.15e}, {y:23.15e}, {z:23.15e}{comma}\n")
        file.write(f"    }};\n")
        file.write(f"\n")

        file.write(f"    static const Eigen::Map<const Eigen::Matrix<double, {rows}, {cols}, Eigen::RowMajor>> CLEAN_DATA(raw_data.data());\n")
        file.write("}\n")


if __name__ == "__main__":
    # Reference: See analysis script
    NUM_COORDINATES = 50
    RADIUS          = 1
    FILENAME        = "clean_data.hpp"


    points = generate_clean_magnetometer_data(NUM_COORDINATES, radius=RADIUS)
    points_flattened = points.reshape(-1, 3)

    script_dir = pathlib.Path(__file__).parent.resolve()
    data_dir = script_dir.parent
    filepath = data_dir / FILENAME

    save_data_as_cpp_header(points_flattened, filepath, radius=RADIUS)
