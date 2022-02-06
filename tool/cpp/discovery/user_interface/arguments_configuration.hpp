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

#ifndef _DDSROUTER_TOOL_CPP_DISCOVERY_USERINTERFACE_ARGUMENTSCONFIGURATION_HPP_
#define _DDSROUTER_TOOL_CPP_DISCOVERY_USERINTERFACE_ARGUMENTSCONFIGURATION_HPP_

#include <string>

#include <optionparser.h>

#include <ddsrouter/types/Time.hpp>

#include "ProcessReturnCode.hpp"

namespace eprosima {
namespace ddsrouter {
namespace discovery {
namespace ui {

constexpr const char* DEFAULT_CONFIGURATION_FILE_NAME = "DDS_ROUTER_DISCOVERY_CONFIGURATION.yaml";

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
};

/*
 * Option arguments available
 */
enum  optionIndex
{
    UNKNOWN_OPT,
    HELP,
    CONFIGURATION_FILE,
    ACTIVATE_DEBUG,
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
        bool& activate_debug);

} /* namespace ui */
} /* namespace discovery */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TOOL_CPP_DISCOVERY_USERINTERFACE_ARGUMENTSCONFIGURATION_HPP_ */
