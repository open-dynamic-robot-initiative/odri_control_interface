#pragma once

#include <boost/python.hpp>
#include "odri_control_interface/calibration.hpp"
#include "odri_control_interface/imu.hpp"
#include "odri_control_interface/joint_modules.hpp"
#include "odri_control_interface/robot.hpp"
#include "odri_control_interface/utils.hpp"

/* make boost::python understand std::shared_ptr */
namespace boost
{
template <typename T>
T *get_pointer(std::shared_ptr<T> p)
{
    return p.get();
}
}  // namespace boost