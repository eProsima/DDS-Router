// Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file arguments_configuration.hpp
 *
 */

#ifndef EPROSIMA_DDSROUTER_USERINTERFACE_ARGUMENTSCONFIGURATION_HPP
#define EPROSIMA_DDSROUTER_USERINTERFACE_ARGUMENTSCONFIGURATION_HPP

#include <string>

#include <optionparser.h>

#include <cpp_utils/macros/custom_enumeration.hpp>
#include <cpp_utils/time/time_utils.hpp>

#include "ProcessReturnCode.hpp"

namespace eprosima {
namespace ddsrouter {
namespace ui {

/*
 * Struct to parse the executable arguments
 */
struct Arg : public option::Arg
{
    //! Print generic error message
    static void print_error(
            const char* msg1,
            const option::Option& opt,
            const char* msg2);

    //! Print error message when argument type is not known
    static option::ArgStatus Unknown(
            const option::Option& option,
            bool msg);

    //! Check that the argument is set
    static option::ArgStatus Required(
            const option::Option& option,
            bool msg);

    //! Check that the argument has integer numeric value
    static option::ArgStatus Numeric(
            const option::Option& option,
            bool msg);

    //! Check that the argument has float (or int) numeric value
    static option::ArgStatus Float(
            const option::Option& option,
            bool msg);

    //! Check that the argument is a string
    static option::ArgStatus String(
            const option::Option& option,
            bool msg);

    //! Check that the argument is a readable file and can be accessed
    static option::ArgStatus Readable_File(
            const option::Option& option,
            bool msg);

    //! Check that the argument is an option of kind
    static option::ArgStatus Log_Kind_Correct_Argument(
            const option::Option& option,
            bool msg);

    static option::ArgStatus Valid_Options(
            const std::vector<std::string>& valid_options,
            const option::Option& option,
            bool msg);
};

/*
 * Option arguments available
 */
enum optionIndex
{
    UNKNOWN_OPT,
    HELP,
    CONFIGURATION_FILE,
    RELOAD_TIME,
    ACTIVATE_DEBUG,
    VERSION,
    TIMEOUT,
    LOG_FILTER,
    LOG_VERBOSITY,
};

/**
 * Usage description
 *
 * @note : Extern used to initialize it in source file
 */
extern const option::Descriptor usage[];

/**
 * @brief Parse process arguments
 *
 * Set variables given as arguments with the arguments given to the process
 *
 * @param [in] argc number of process arguments
 * @param [in] argv process arguments array (with size \c argc )
 * @param [out] file_path path to the configuration file
 * @param [out] reload_time time in milliseconds to reload the configuration file
 * @param [out] activate_debug activate log info
 * @param [out] timeout time in milliseconds to maximum router execution time
 *
 * @return \c SUCCESS if everything OK
 * @return \c INCORRECT_ARGUMENT if arguments were incorrect (unknown or incorrect value)
 * @return \c HELP_ARGUMENT if arguments help given
 * @return \c REQUIRED_ARGUMENT_FAILED if required arguments not given
 */
ProcessReturnCode parse_arguments(
        int argc,
        char** argv,
        std::string& file_path,
        utils::Duration_ms& reload_time,
        utils::Duration_ms& timeout,
        std::string& log_filter,
        eprosima::fastdds::dds::Log::Kind& log_verbosity);

//! \c Option to stream serializator
std::ostream& operator <<(
        std::ostream& output,
        const option::Option& option);

/**
 * @brief Print version in console.
 */
void print_version();

ENUMERATION_BUILDER(
    LogKind,
    error,
    warning,
    info
    );

} /* namespace ui */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* EPROSIMA_DDSROUTER_USERINTERFACE_ARGUMENTSCONFIGURATION_HPP */
