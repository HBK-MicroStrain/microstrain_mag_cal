#pragma once

#include <Eigen/Dense>

#include <microstrain/platform.h>
#include <mip/definitions/data_sensor.hpp>
#include <mip/mip_parser.hpp>

using ScaledMag = mip::data_sensor::ScaledMag;

namespace Core
{
    Eigen::MatrixX3d extractPointMatrixFromRawData(const uint8_t *data, const size_t data_size);
}
