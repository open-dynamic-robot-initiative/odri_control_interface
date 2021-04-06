/**
 * @file device.hpp
 * @author Julian Viereck (jviereck@tuebingen.mpg.de)
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft.
 * @date 2020-11-27
 *
 * @brief Abstract device class used in framework.
 */

#pragma once

#include <unistd.h>

#include "master_board_sdk/defines.h"
#include "master_board_sdk/master_board_interface.h"

namespace odri_control_interface
{
/**
 * @brief Abstract device class used in framework.
 */
class Device
{
public:
    virtual bool HasError() = 0;
};

}  // namespace odri_control_interface
