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
 * @file arguments_configuration.cpp
 *
 */

#include <iostream>
#include <string>
#include <vector>

#if defined(_WIN32)
#include <io.h>         // Use _access windows method
#define access _access  // Use access method as windows _access method
#define R_OK 04         // Use R_OK variable as Redeable for _access
#else
#include <unistd.h>
#endif // if defined(_WIN32)

#include "arguments_configuration.hpp"

namespace eprosima {
namespace ddsrouter {
namespace ui {

const option::Descriptor usage[] = {
    {
        optionIndex::UNKNOWN_OPT,
        0,
        "",
        "",
        Arg::None,
        "Usage: Fast DDS Router \n" \
        "Connect different DDS networks via DDS through LAN or WAN.\n" \
        "It will build a communication bridge between the different " \
        "Participants included in the provided configuration file.\n" \
        "To close the execution gracefully use SIGINT (C^) or SIGTERM (kill).\n" \
        "General options:"
    },

    {
        optionIndex::HELP,
        0,
        "h",
        "help",
        Arg::None,
        "  -h \t--help\t  \t" \
        "Print this help message."
    },

    {
        optionIndex::CONFIGURATION_FILE,
        0,
        "c",
        "config-path",
        Arg::Readable_File,
        "  -c \t--config-path\t  \t" \
        "Path to the Configuration File (yaml format) [Default: ./DDS_ROUTER_CONFIGURATION.yaml]."
    },

    {
        optionIndex::RELOAD_TIME,
        0,
        "r",
        "reload",
        Arg::Numeric,
        "  -r \t--reload-time\t  \t" \
        "Time period in seconds to reload configuration file. " \
        "This is needed when FileWatcher functionality is not available (e.g. config file is a symbolic link). " \
        "Value 0 does not reload file. [Default: 0]."
    },

    {
        optionIndex::ACTIVATE_DEBUG,
        0,
        "d",
        "debug",
        Arg::None,
        "  -d \t--debug\t  \t" \
        "Activate debug Logs (be aware that some logs may require specific CMAKE compilation options)." \
    },

    { 0, 0, 0, 0, 0, 0 }
};

ProcessReturnCode parse_arguments(
        int argc,
        char** argv,
        std::string& file_path,
        utils::Duration_ms& reload_time,
        bool& activate_debug)
{
    // Variable to pretty print usage help
    int columns;
#if defined(_WIN32)
    char* buf = nullptr;
    size_t sz = 0;
    if (_dupenv_s(&buf, &sz, "COLUMNS") == 0 && buf != nullptr)
    {
        columns = std::strtol(buf, nullptr, 10);
        free(buf);
    }
    else
    {
        columns = 80;
    }
#else
    columns = getenv("COLUMNS") ? atoi(getenv("COLUMNS")) : 180;
#endif // if defined(_WIN32)

    // Parse arguments
    // No required arguments
    if (argc > 0)
    {
        argc -= (argc > 0); // reduce arg count of program name if present
        argv += (argc > 0); // skip program name argv[0] if present

        option::Stats stats(usage, argc, argv);
        std::vector<option::Option> options(stats.options_max);
        std::vector<option::Option> buffer(stats.buffer_max);
        option::Parser parse(usage, argc, argv, &options[0], &buffer[0]);

        // Parsing error
        if (parse.error())
        {
            option::printUsage(fwrite, stdout, usage, columns);
            return ProcessReturnCode::INCORRECT_ARGUMENT;
        }

        // Unknown args provided
        if (parse.nonOptionsCount())
        {
            std::cerr << "ERROR: Unknown argument: <" << parse.nonOption(0) << ">." << std::endl;
            option::printUsage(fwrite, stdout, usage, columns);
            return ProcessReturnCode::INCORRECT_ARGUMENT;
        }

        // Adding Help before every other check to show help in case an argument is incorrect
        if (options[optionIndex::HELP])
        {
            option::printUsage(fwrite, stdout, usage, columns);
            return ProcessReturnCode::HELP_ARGUMENT;
        }

        for (int i = 0; i < parse.optionsCount(); ++i)
        {
            option::Option& opt = buffer[i];
            switch (opt.index())
            {
                case optionIndex::CONFIGURATION_FILE:
                    file_path = opt.arg;
                    break;

                case optionIndex::RELOAD_TIME:
                    reload_time = std::stol(opt.arg) * 1000; // pass to milliseconds
                    break;

                case optionIndex::ACTIVATE_DEBUG:
                    activate_debug = true;
                    break;

                case optionIndex::UNKNOWN_OPT:
                    Arg::print_error("ERROR: ", opt, " is not a valid argument.\n");
                    option::printUsage(fwrite, stdout, usage, columns);
                    return ProcessReturnCode::INCORRECT_ARGUMENT;
                    break;

                default:
                    break;
            }
        }
    }
    else
    {
        option::printUsage(fwrite, stdout, usage, columns);
        return ProcessReturnCode::INCORRECT_ARGUMENT;
    }

    return ProcessReturnCode::SUCCESS;
}

void Arg::print_error(
        const char* msg1,
        const option::Option& opt,
        const char* msg2)
{
    fprintf(stderr, "%s", msg1);
    fwrite(opt.name, opt.namelen, 1, stderr);
    fprintf(stderr, "%s", msg2);
}

option::ArgStatus Arg::Unknown(
        const option::Option& option,
        bool msg)
{
    if (msg)
    {
        print_error("Unknown option '", option, "'\nUse -h to see this executable possible arguments.\n");
    }
    return option::ARG_ILLEGAL;
}

option::ArgStatus Arg::Required(
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

option::ArgStatus Arg::Numeric(
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

option::ArgStatus Arg::Float(
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

option::ArgStatus Arg::String(
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

option::ArgStatus Arg::Readable_File(
        const option::Option& option,
        bool msg)
{
    if (option.arg != 0)
    {
        // Windows has not unistd library, so to check if file is readable use a _access method (definition on top)
        if (access( option.arg, R_OK ) != -1)
        {
            return option::ARG_OK;
        }
    }
    if (msg)
    {
        print_error("Option '", option, "' requires an existing readable file as argument\n");
    }
    return option::ARG_ILLEGAL;
}

} /* namespace ui */
} /* namespace ddsrouter */
} /* namespace eprosima */
