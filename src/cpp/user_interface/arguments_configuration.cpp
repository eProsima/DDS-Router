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

#include <vector>

#include <ddsrouter/user_interface/arguments_configuration.hpp>
#include <ddsrouter/types/Log.hpp>

namespace eprosima {
namespace ddsrouter {
namespace ui {

ProcessReturnCode parse_arguments(
    int argc,
    char** argv,
    std::string& file_path,
    eprosima::ddsrouter::Duration_ms& reload_time,
    bool& active_debug)
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

        if (parse.error())
        {
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
                    reload_time = std::stol(opt.arg);
                    break;

                case optionIndex::ACTIVE_DEBUG:
                    active_debug = true;
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

} /* namespace ui */
} /* namespace ddsrouter */
} /* namespace eprosima */
