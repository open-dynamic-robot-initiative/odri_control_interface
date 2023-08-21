/**
 * @file
 * @brief TODO
 * @copyright 2020 New York University & Max Planck Gesellschaft.
 * @license BSD-3-Clause
 */
#pragma once

#include <memory>
#include <string>

namespace odri_control_interface
{
/**
 * @brief Base class for errors.
 */
class Error
{
public:
    typedef std::shared_ptr<Error> Ptr;
    typedef std::shared_ptr<const Error> ConstPtr;

    //! Get a human-readable error message.
    virtual std::string get_message() const = 0;
};

}  // namespace odri_control_interface
