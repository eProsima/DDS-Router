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

#include <ddsrouter_utils/Log.hpp>
#include <ddsrouter_utils/utils.hpp>

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
        "To stop the execution gracefully use SIGINT (C^) or SIGTERM (kill) signals.\n" \
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
        "reload-time",
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
    {
        optionIndex::VERSION,
        0,
        "v",
        "version",
        Arg::None,
        "  -v \t--version\t  \t" \
        "Print version, branch and commit hash." \
    },

    { 0, 0, 0, 0, 0, 0 }
};

ProcessReturnCode parse_arguments(
        int argc,
        char** argv,
        std::string& file_path,
        utils::Duration_ms& reload_time,
        bool& activate_debug,
        bool& print_version)
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
            return ProcessReturnCode::incorrect_argument;
        }

        // Unknown args provided
        if (parse.nonOptionsCount())
        {
            logError(DDSROUTER_ARGS, "ERROR: Unknown argument: <" << parse.nonOption(0) << ">." );
            option::printUsage(fwrite, stdout, usage, columns);
            return ProcessReturnCode::incorrect_argument;
        }

        // Adding Help before every other check to show help in case an argument is incorrect
        if (options[optionIndex::HELP])
        {
            option::printUsage(fwrite, stdout, usage, columns);
            return ProcessReturnCode::help_argument;
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

                case optionIndex::VERSION:
                    print_version = true;
                    break;

                case optionIndex::UNKNOWN_OPT:
                    logError(DDSROUTER_ARGS, opt << " is not a valid argument.");
                    option::printUsage(fwrite, stdout, usage, columns);
                    return ProcessReturnCode::incorrect_argument;
                    break;

                default:
                    break;
            }
        }
    }
    else
    {
        option::printUsage(fwrite, stdout, usage, columns);
        return ProcessReturnCode::incorrect_argument;
    }

    return ProcessReturnCode::success;
}

option::ArgStatus Arg::Unknown(
        const option::Option& option,
        bool msg)
{
    if (msg)
    {
        logError(
            DDSROUTER_ARGS,
            "Unknown option '" << option << "'. Use -h to see this executable possible arguments.");
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
        logError(DDSROUTER_ARGS, "Option '" << option << "' required.");
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
        logError(DDSROUTER_ARGS, "Option '" << option << "' requires a numeric argument.");
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
        logError(DDSROUTER_ARGS, "Option '" << option << "' requires a float argument.");
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
        logError(DDSROUTER_ARGS, "Option '" << option << "' requires a text argument.");
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
        if (is_file_accessible(option.arg, utils::FileAccessMode::read))
        {
            return option::ARG_OK;
        }
    }
    if (msg)
    {
        logError(DDSROUTER_ARGS, "Option '" << option << "' requires an existing readable file as argument.");
    }
    return option::ARG_ILLEGAL;
}

std::ostream& operator <<(
        std::ostream& output,
        const option::Option& option)
{
    output << std::string(option.name, option.name + option.namelen);
    return output;
}

} /* namespace ui */
} /* namespace ddsrouter */
} /* namespace eprosima */
