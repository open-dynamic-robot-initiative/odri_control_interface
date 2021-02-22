/**
 * @file common.hpp
 * @author Julian Viereck (jviereck@tuebingen.mpg.de)
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft.
 * @date 2020-11-27
 *
 * @brief Common definitions used across the library.
 */

#include <Eigen/Dense>

namespace odri_control_interface
{
typedef Eigen::VectorXd VectorXd;
typedef Eigen::Matrix<int, Eigen::Dynamic, 1> VectorXi;
typedef Eigen::Matrix<long, Eigen::Dynamic, 1> VectorXl;
typedef Eigen::Matrix<bool, Eigen::Dynamic, 1> VectorXb;

typedef Eigen::Ref<Eigen::Matrix<int, Eigen::Dynamic, 1> > RefVectorXi;
typedef Eigen::Ref<Eigen::Matrix<long, Eigen::Dynamic, 1> > RefVectorXl;
typedef Eigen::Ref<Eigen::Matrix<bool, Eigen::Dynamic, 1> > RefVectorXb;
typedef Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1> > RefVectorXd;
typedef Eigen::Ref<Eigen::Vector3d> RefVector3d;
typedef Eigen::Ref<Eigen::Vector4d> RefVector4d;

typedef const Eigen::Ref<const Eigen::Matrix<int, Eigen::Dynamic, 1> >
    ConstRefVectorXi;
typedef const Eigen::Ref<const Eigen::Matrix<long, Eigen::Dynamic, 1> >
    ConstRefVectorXl;
typedef const Eigen::Ref<const Eigen::Matrix<bool, Eigen::Dynamic, 1> >
    ConstRefVectorXb;
typedef const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1> >
    ConstRefVectorXd;
typedef const Eigen::Ref<const Eigen::Vector3d> ConstRefVector3d;
typedef const Eigen::Ref<const Eigen::Vector4d> ConstRefVector4d;

}  // namespace odri_control_interface
