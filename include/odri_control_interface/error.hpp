/**
 * @file
 * @brief TODO
 * @copyright 2020 New York University & Max Planck Gesellschaft.
 * @license BSD-3-Clause
 */
#pragma once

#include <memory>
#include <string>

#include <array>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include <fmt/format.h>

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

// `Message` is based on https://stackoverflow.com/a/76944227/2095383
// by Aedoro, CC BY-SA 4.0
// NOTE: below code is adapted to work with C++17.  If this package ever gets
// updated to C++20, it might be worth revisiting this and put back the
// concept/requires-part of the answer linked above.

using MessageParam = std::variant<int, double>;

// FIXME: rename variables and maybe clean up a bit
// FIXME: can I get the argument limit working?
// FIXME: write some doxygen
// FIXME: actually use it instead of the classes based on Error.  remove the
//        latter
template <int MAX_PARAM>
class Message
{
public:
    template <typename... Args,
             // ensure number or arguments does not exceed MAX_PARAM
             // https://stackoverflow.com/a/39621288/2095383
             // NOTE: With C++20, it would be better to use `requires`
              std::enable_if_t<(sizeof...(Args) <= MAX_PARAM)>* = nullptr>
    Message(std::string_view format, Args... args)
        : m_format(format), m_nr_params(sizeof...(args)), m_params{args...}
    {
    }

    auto get_message() const
    {
        // constructing an argument list for fmt::format from an array is a bit
        // of a pain... based on https://stackoverflow.com/a/59744762
        // NOTE: for newer versions of fmt there are better options, see
        // https://stackoverflow.com/a/71794255 and maybe
        // https://stackoverflow.com/a/48877448

        using fmt_ctx = fmt::format_context;
        std::vector<fmt::basic_format_arg<fmt_ctx>> fmt_args;
        for (auto const& a : m_params)
        {
            if (std::holds_alternative<double>(a))
            {
                fmt_args.push_back(
                    fmt::internal::make_arg<fmt_ctx>(std::get<double>(a)));
            }
            else if (std::holds_alternative<int>(a))
            {
                fmt_args.push_back(
                    fmt::internal::make_arg<fmt_ctx>(std::get<int>(a)));
            }
        }

        return fmt::vformat(
            m_format,
            fmt::basic_format_args<fmt_ctx>(fmt_args.data(), fmt_args.size()));
    }

private:
    std::string_view m_format;
    size_t m_nr_params;
    std::array<MessageParam, MAX_PARAM> m_params;
};

// Allow up to three arguments for error messages (increase number if needed)
using ErrorMessage = Message<3>;

}  // namespace odri_control_interface
