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
#include <unistd.h>

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
    static void print_error(
            const char* msg1,
            const option::Option& opt,
            const char* msg2)
    {
        fprintf(stderr, "%s", msg1);
        fwrite(opt.name, opt.namelen, 1, stderr);
        fprintf(stderr, "%s", msg2);
    }

    static option::ArgStatus Unknown(
            const option::Option& option,
            bool msg)
    {
        if (msg)
        {
            print_error("Unknown option '", option, "'\nUse -h to see this executable possible arguments.\n");
        }
        return option::ARG_ILLEGAL;
    }

    static option::ArgStatus Required(
            const option::Option& option,
            bool msg)
    {
        if (option.arg != 0 && option.arg[0] != 0)
        {
            return option::ARG_OK;
        }

        if (msg)
        {
            print_error("Option '", option, "' requires an argument\n");
        }
        return option::ARG_ILLEGAL;
    }

    static option::ArgStatus Numeric(
            const option::Option& option,
            bool msg)
    {
        char* endptr = 0;
        if (option.arg != 0 && std::strtol(option.arg, &endptr, 10))
        {
        }
        if (endptr != option.arg && *endptr == 0)
        {
            return option::ARG_OK;
        }

        if (msg)
        {
            print_error("Option '", option, "' requires a numeric argument\n");
        }
        return option::ARG_ILLEGAL;
    }

    static option::ArgStatus Float(
            const option::Option& option,
            bool msg)
    {
        char* endptr = 0;
        if (option.arg != 0 && std::strtof(option.arg, &endptr))
        {
        }
        if (endptr != option.arg && *endptr == 0)
        {
            return option::ARG_OK;
        }

        if (msg)
        {
            print_error("Option '", option, "' requires a float argument\n");
        }
        return option::ARG_ILLEGAL;
    }

    static option::ArgStatus String(
            const option::Option& option,
            bool msg)
    {
        if (option.arg != 0)
        {
            return option::ARG_OK;
        }
        if (msg)
        {
            print_error("Option '", option, "' requires a text argument\n");
        }
        return option::ARG_ILLEGAL;
    }

    static option::ArgStatus File(
            const option::Option& option,
            bool msg)
    {
        if (option.arg != 0)
        {
            if (access( option.arg, F_OK ) != -1)
            {
                return option::ARG_OK;
            }
        }
        if (msg)
        {
            print_error("Option '", option, "' requires an existing file as argument\n");
        }
        return option::ARG_ILLEGAL;
    }
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

/*
 * Usage description
 */
const option::Descriptor usage[] = {
    {
        optionIndex::UNKNOWN_OPT,
        0,
        "",
        "",
        Arg::None,
        "Usage: Fast DDS Router \n" \
        "Connect different DDS networks via DDS through LAN or WAN.\n" \
        "It will build a bridge between the different Participant configurations set.\n" \
        "General options:"
    },

    {
        optionIndex::HELP,
        0,
        "h",
        "help",
        Arg::None,
        "  -h \t--help\t  \t" \
        "Produce this help message."
    },

    {
        optionIndex::CONFIGURATION_FILE,
        0,
        "f",
        "config-path",
        Arg::File,
        "  -f \t--config-path\t  \t" \
        "Path to the Configuration File (yaml format) [Default: ./DDS_ROUTER_CONFIGURATION.yaml]."
    },

    {
        optionIndex::RELOAD_TIME,
        0,
        "r",
        "reload",
        Arg::Numeric,
        "  -r \t--reload\t  \t" \
        "Time period in miliseconds to reload configuration file. " \
        "This is needed when FileWatcher functionality is not available (config file is a symbolic link). " \
        "Value 0 does not reload file. [Default: 0]."
    },

    {
        optionIndex::ACTIVE_DEBUG,
        0,
        "d",
        "debug",
        Arg::None,
        "  -d \t--debug\t  \t" \
        "Active debug Logs. (Be aware that some logs may require to have compiled with specific CMAKE options)"
    },

    { 0, 0, 0, 0, 0, 0 }
};

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
