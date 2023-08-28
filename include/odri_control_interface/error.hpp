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

using MessageArgument = std::variant<int, double, std::string_view>;

/**
 * @brief Generic message class with lazy message construction.
 *
 * The constructor only takes a format string (for the fmt library) as
 * string_view and arguments for it but does not directly construct the actual
 * message from it.  This means no dynamic memory allocation is happening during
 * construction (so safe to use in real-time critical code). The actual message
 * string is only generated when calling @ref get_message.
 *
 * Based on https://stackoverflow.com/a/76944227 by Aedoro, CC BY-SA 4.0
 *
 * @tparam MAX_ARGS Maximum number of arguments that can be used in the format
 * string.
 */
template <int MAX_ARGS>
class Message
{
public:
    /**
     * @brief Constructor
     *
     * @param format Format string with '{}' placeholders (will be processed
     *      with the fmt library, see there for more information).
     * @param args Arguments for the format string.  Only types that are
     *      convertible to MessageArgument are supported.
     */
    template <typename... Args,
              // ensure number or arguments does not exceed MAX_ARGS
              // https://stackoverflow.com/a/39621288
              // NOTE: With C++20, it would be better to use `requires`
              std::enable_if_t<(sizeof...(Args) <= MAX_ARGS)>* = nullptr>
    Message(std::string_view format, Args... args)
        : format_(format), num_args_(sizeof...(args)), args_{args...}
    {
    }

    /**
     * @brief Construct the message from format string and arguments.
     *
     * This involves dynamic memory allocation so it is not real-time safe!
     */
    std::string get_message() const
    {
        // constructing an argument list for fmt::format from an array is a bit
        // of a pain... based on https://stackoverflow.com/a/59744762
        // NOTE: for newer versions of fmt there are better options, see
        // https://stackoverflow.com/a/71794255 and maybe
        // https://stackoverflow.com/a/48877448

        using fmt_ctx = fmt::format_context;
        std::vector<fmt::basic_format_arg<fmt_ctx>> fmt_args;
        for (auto const& a : args_)
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
            format_,
            fmt::basic_format_args<fmt_ctx>(fmt_args.data(), fmt_args.size()));
    }

private:
    //! Format string for the message (will be filled with args_ using fmt)
    std::string_view format_;
    //! Number of arguments that where passed to the constructor.
    size_t num_args_;
    //! Message arguments.  Elements with index >= num_args_ are undefined.
    std::array<MessageArgument, MAX_ARGS> args_;
};

// Allow up to three arguments for error messages (increase number if needed)
using ErrorMessage = Message<3>;

}  // namespace odri_control_interface
