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
 * @file arg_configuration.hpp
 *
 */

#ifndef EPROSIMA_DDSROUTER_USERINTERFACE_ARGCONFIURATION_HPP
#define EPROSIMA_DDSROUTER_USERINTERFACE_ARGCONFIURATION_HPP

#include <string>

#include <optionparser.h>

#include <ddsrouter/types/Time.hpp>
#include <ddsrouter/user_interface/ProcessReturnCode.hpp>

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

    //! Check that the argument is a redeable file with access permissions
    static option::ArgStatus Readable_File(
            const option::Option& option,
            bool msg);
};

/*
 * Option arguments available
 */
enum  optionIndex
{
    UNKNOWN_OPT,
    HELP,
    CONFIGURATION_FILE,
    RELOAD_TIME,
    ACTIVE_DEBUG,
};

/**
 * Usage description
 *
 * @note : Extern used to initialize it in source file
 */
extern const option::Descriptor usage[];

ProcessReturnCode parse_arguments(
    int argc,
    char** argv,
    std::string& file_path,
    eprosima::ddsrouter::Duration_ms& reload_time,
    bool& active_debug);

} /* namespace ui */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* EPROSIMA_DDSROUTER_USERINTERFACE_ARGCONFIURATION_HPP */
