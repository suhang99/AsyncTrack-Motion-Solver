#pragma once

#include <Eigen/Dense>

namespace motion_solver {

using Scalar = double;
using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
using Vec4 = Eigen::Matrix<Scalar, 4, 1>;
using Mat2 = Eigen::Matrix<Scalar, 2, 2>;
using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
using Mat4 = Eigen::Matrix<Scalar, 4, 4>;
using Pixel = Eigen::Matrix<Scalar, 2, 1>;

using Twist = std::tuple<Vec3, Vec3, Eigen::AngleAxisd>;
using Pose = Eigen::Isometry3d;
using StampedPixel = std::pair<Scalar, Pixel>;

}