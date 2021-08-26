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
 * @file DataBroker.cpp
 *
 */

#include <algorithm>
#include <iostream>
#include <string>

#include <fastdds/dds/log/Log.hpp>

#include <databroker/arg_configuration.hpp>
#include <databroker/Address.hpp>
#include <databroker/DataBroker.hpp>
#include <databroker/DataBrokerConfiguration.hpp>
#include <databroker/utils.hpp>

using namespace eprosima;
using namespace databroker;

int main(
        int argc,
        char* argv[])
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

    eprosima::fastdds::dds::Log::SetVerbosity(eprosima::fastdds::dds::Log::Kind::Info);

    DataBrokerConfiguration configuration;

    // Check if default configuration file exists and stablish default input values
    DataBrokerConfiguration::load_default_configuration(configuration);

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
            return 4;
        }

        if (options[optionIndex::HELP])
        {
            option::printUsage(fwrite, stdout, usage, columns);
            return 0;
        }

        for (int i = 0; i < parse.optionsCount(); ++i)
        {
            option::Option& opt = buffer[i];
            switch (opt.index())
            {
                case optionIndex::TIME:
                    configuration.seconds = std::stol(opt.arg);
                    break;

                case optionIndex::SERVER_ID:
                    configuration.server_guid = Address::guid_server(std::stol(opt.arg));
                    break;

                case optionIndex::SERVER_GUID:
                    configuration.server_guid = Address::guid_server(opt.arg);
                    break;

                case optionIndex::WHITELIST:
                    if (!utils::split_string(opt.arg, configuration.active_topics))
                    {
                        std::cerr << "Error parsing whitelist" << std::endl;
                        return 10;
                    }
                    break;

                case optionIndex::UDP:
                    configuration.udp = true;
                    break;

                case optionIndex::DOMAIN:
                    configuration.domain = std::stol(opt.arg);
                    break;

                case optionIndex::ROS:
                    configuration.ros = true;
                    break;

                case optionIndex::LISTENING_ADDRESSES:
                    configuration.listening_addresses.clear();
                    if (!Address::read_addresses_vector(opt.arg, configuration.listening_addresses))
                    {
                        std::cerr << "Error parsing listening addreses" << std::endl;
                        return 10;
                    }
                    break;

                case optionIndex::CONNECTION_ADDRESSES:
                    configuration.connection_addresses.clear();
                    if (!Address::read_addresses_vector(opt.arg, configuration.connection_addresses))
                    {
                        std::cerr << "Error parsing connection addreses" << std::endl;
                        return 10;
                    }
                    break;

                case optionIndex::INTERACTIVE:
                    configuration.interactive = true;
                    break;

                case optionIndex::UNKNOWN_OPT:
                    option::printUsage(fwrite, stdout, usage, columns);
                    return 5;
                    break;
            }
        }
    }
    else
    {
        option::printUsage(fwrite, stdout, usage, columns);
        return 3;
    }

    // TODO add warning messages to input

    // Create DataBroker instance
    // Addresses cannot be modified in run time, so they are set in construction
    DataBroker db(
        configuration.domain,
        configuration.server_guid,
        configuration.listening_addresses,
        configuration.connection_addresses,
        configuration.ros,
        configuration.udp);

    // Configure DataBroker
    if (!db.init(configuration.active_topics))
    {
        std::cerr << "Error initializing DataBroker" << std::endl;
        return 2;
    }

    // Run DataBroker instance
    if(db.run(configuration.interactive, configuration.seconds))
    {
        return 0;
    }
    else
    {
        return 1;
    }
}
