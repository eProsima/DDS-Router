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

#include <optionparser.h>

#include <fastdds/dds/log/Log.hpp>

#include <databroker/Address.hpp>
#include <databroker/DataBroker.hpp>

using namespace eprosima;
using namespace databroker;

static const std::regex ipv4_port_id("(^(((((([0-9]{1,3}\\.){1,3})([0-9]{1,3})),([0-9]{1,5}),([0-9]+));)*"
                              "((((([0-9]{1,3}\\.){1,3})([0-9]{1,3})),([0-9]{1,5}),([0-9]+))?))$)");

static const std::regex ipv4_port("(^(((((([0-9]{1,3}\\.){1,3})([0-9]{1,3})),([0-9]{1,5}));)*"
                              "(((([0-9]{1,3}\\.){1,3})([0-9]{1,3})),([0-9]{1,5})))$)");

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
            print_error("Unknown option '", option, "'\nUse -h to see this execuable possible arguments.\n");
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
            print_error("Option '", option, "' requires a numeric argument\n");
        }
        return option::ARG_ILLEGAL;
    }

    static option::ArgStatus Locator(
            const option::Option& option,
            bool msg)
    {
        if (option.arg != 0)
        {
            // we must check if its a correct ip address plus port number
            if (std::regex_match(option.arg, ipv4_port))
            {
                return option::ARG_OK;
            }
        }
        if (msg)
        {
            print_error("Option '", option, "' requires an ip,port[;ip,port[;...]] argument\n");
        }
        return option::ARG_ILLEGAL;
    }

    static option::ArgStatus DSLocator(
            const option::Option& option,
            bool msg)
    {
        if (option.arg != 0)
        {
            // we must check if its a correct ip address plus port number
            if (std::regex_match(option.arg, ipv4_port_id))
            {
                return option::ARG_OK;
            }
        }
        if (msg)
        {
            print_error("Option '", option, "' requires an ip,port,id[;ip,port,id[;...]] argument\n");
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
    TIME,
    SERVER_ID,
    SERVER_GUID,
    WHITELIST,
    UDP,
    DOMAIN,
    ROS,
    LISTENING_ADDRESSES,
    CONNECTION_ADDRESSES,
    INTERACTIVE,
};

/*
 * Usage description
 */
const option::Descriptor usage[] = {
    { UNKNOWN_OPT, 0, "", "",                Arg::None,
      "Usage: Fast DataBroker \n" \
      "Connect two different DDS networks via DDS.\n" \
      "It will build a bridge between its internal network and a WAN range network.\n" \
      "General options:" },

    {
        HELP,
        0,
        "h",
        "help",
        Arg::None,
        "  -h \t--help\t  \t" \
        "Produce this help message."
    },

    {
        TIME,
        0,
        "t",
        "time",
        Arg::Numeric,
        "  -t \t--time\t<uint>  \t" \
        "Time in seconds that the DataBroker will run before shutting down. " \
        "With interactive mode active, this argument will not be used. " \
        "With argument 0 will run forever. "\
        "Default: 0"
    },

    {
        SERVER_ID,
        0,
        "s",
        "server-id",
        Arg::Numeric,
        "  -s \t--server-id\t<uint_8>  \t" \
        "Id seed to create the server guid for the external conecction. " \
        "In case server-guid is also set, the last one will be chosen. " \
        "Default 0"
    },

    {
        SERVER_GUID,
        0,
        "s",
        "server-guid",
        Arg::String,
        "  -g \t--server-guid\t<string>  \t" \
        "GUID to the server for the external conecction. " \
        "In case server-id is also set, the last one will be chosen. " \
        "Format: 12 two hexadecimal digit numbers separated by points. " \
        "Default: " SERVER_DEFAULT_GUID
    },

    {
        WHITELIST,
        0,
        "w",
        "whitelist",
        Arg::String,
        "  -w \t--whitelist\t<string>  \t" \
        "List of topics that will be communicated from the beggining. " \
        "Format: name of topics without spaces separated by ';'. " \
        "Default: none"
    },

    {
        UDP,
        0,
        "u",
        "udp",
        Arg::Optional,
        "  -u \t--udp\t  \t" \
        "Use UDP instead of TCP transport."
    },

    {
        DOMAIN,
        0,
        "d",
        "domain",
        Arg::Numeric,
        "  -d \t--domain\t<uint>  \t" \
        "Domain ID for the local Participant. " \
        "Default: 0"
    },

    {
        ROS,
        0,
        "r",
        "ros",
        Arg::Optional,
        "  -r \t--ros\t  \t" \
        "Mangling topic and type to connect with an internal ROS 2 network."
    },

    {
        LISTENING_ADDRESSES,
        0,
        "l",
        "listening-addresses",
        Arg::Locator,
        "  -l \t--listening-addresses\t<string>  \t" \
        "List of tuples 'ip,port' for listening addresses. " \
        "External Participant will use Discovery Server Protocol and it needs listening addresses. " \
        "In order to communicate through WAN, use the public address and public port. " \
        "This argument is required. Otherwise the external Participant will not be able to connect. " \
        "Format: list of tuples 'ip,port' (separated by ',') separated by ';'. " \
        "Default: '127.0.0.1,11800'"
    },

    {
        CONNECTION_ADDRESSES,
        0,
        "c",
        "connection-addresses",
        Arg::Locator,
        "  -c \t--connection-addresses\t<string>  \t" \
        "List of tuples 'ip,port,server_id' for connection addresses. " \
        "External Participant will use Discovery Server Protocol. In order to connect with remote servers" \
        "is necessary to set an external connection address or use this DataBroker listening address as" \
        "remote connection of other DataBroker. " \
        "Format: list of tuples 'ip,port,server_id' (separated by ',') separated by ';'. " \
        "Default: none"
    },

    {
        INTERACTIVE,
        0,
        "i",
        "interactive",
        Arg::Optional,
        "  -i \t--interactive\t  \t" \
        "Use DataBroker in interactive mode. " \
        "Interactive mode keeps DataBroker listening for user commands. " \
        "These commands are 'help', 'add <topic>', 'remove <topic', 'exit'."
    },

    { 0, 0, 0, 0, 0, 0 }
};

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
    eprosima::fastdds::dds::Log::SetCategoryFilter(std::regex("DATABROKER"));

    std::string listening_addresses_str("127.0.0.1,11800");
    std::string connection_addresses_str;
    std::vector<std::string> active_topics;
    uint32_t seconds = 0;
    bool interactive = false;
    bool ros = false;
    uint32_t domain = 0;
    bool udp = false;
    eprosima::fastrtps::rtps::GuidPrefix_t server_guid = Address::guid_server();

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

        if (options[HELP])
        {
            option::printUsage(fwrite, stdout, usage, columns);
            return 0;
        }

        for (int i = 0; i < parse.optionsCount(); ++i)
        {
            option::Option& opt = buffer[i];
            switch (opt.index())
            {
                case HELP:
                    option::printUsage(fwrite, stdout, usage, columns);
                    return 0;
                    break;

                case TIME:
                    seconds = std::stol(opt.arg);
                    break;

                case SERVER_ID:
                    server_guid = Address::guid_server(std::stol(opt.arg));
                    break;

                case SERVER_GUID:
                    server_guid = Address::guid_server(opt.arg);
                    break;

                case WHITELIST:
                    if (!Address::split_string(opt.arg, active_topics))
                    {
                        std::cerr << "Error parsing whitelist" << std::endl;
                        return 10;
                    }
                    break;

                case UDP:
                    udp = true;
                    break;

                case DOMAIN:
                    domain = std::stol(opt.arg);
                    break;

                case ROS:
                    ros = true;
                    break;

                case LISTENING_ADDRESSES:
                    listening_addresses_str = opt.arg;
                    break;

                case CONNECTION_ADDRESSES:
                    connection_addresses_str = opt.arg;
                    break;

                case UNKNOWN_OPT:
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

    std::vector<Address> listening_addresses = Address::read_addresses_vector(listening_addresses_str);
    std::vector<Address> connection_addresses = Address::read_addresses_vector(connection_addresses_str);

    // TODO add warning messages to input

    // Create DataBroker instance
    // Addresses cannot be modified in run time, so they are set in construction
    DataBroker db(domain, server_guid, listening_addresses, connection_addresses, ros, udp);

    // Configure DataBroker
    if (!db.init(active_topics))
    {
        std::cerr << "Error initializing DataBroker" << std::endl;
        return 2;
    }

    // Run DataBroker instance
    if(db.run(interactive, seconds))
    {
        return 0;
    }
    else
    {
        return 1;
    }
}
