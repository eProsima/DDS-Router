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

#ifndef EPROSIMA_DATABROKER_ARG_CONFIGURATION_HPP
#define EPROSIMA_DATABROKER_ARG_CONFIGURATION_HPP

#include <algorithm>
#include <regex>

#include <optionparser.h>

#include <databroker/Address.hpp>

namespace eprosima {
namespace databroker {


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
    {
        optionIndex::UNKNOWN_OPT,
        0,
        "",
        "",
        Arg::None,
        "Usage: Fast DataBroker \n" \
        "Connect two different DDS networks via DDS.\n" \
        "It will build a bridge between its internal network and a WAN range network.\n" \
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
        optionIndex::TIME,
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
        optionIndex::SERVER_ID,
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
        optionIndex::SERVER_GUID,
        0,
        "g",
        "server-guid",
        Arg::String,
        "  -g \t--server-guid\t<string>  \t" \
        "GUID to the server for the external conecction. " \
        "In case server-id is also set, the last one will be chosen. " \
        "Format: 12 two hexadecimal digit numbers separated by points. " \
        "Default: " SERVER_DEFAULT_GUID
    },

    {
        optionIndex::WHITELIST,
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
        optionIndex::UDP,
        0,
        "u",
        "udp",
        Arg::Optional,
        "  -u \t--udp\t  \t" \
        "Use UDP instead of TCP transport."
    },

    {
        optionIndex::DOMAIN,
        0,
        "d",
        "domain",
        Arg::Numeric,
        "  -d \t--domain\t<uint>  \t" \
        "Domain ID for the local Participant. " \
        "Default: 0"
    },

    {
        optionIndex::ROS,
        0,
        "r",
        "ros",
        Arg::Optional,
        "  -r \t--ros\t  \t" \
        "Mangling topic and type to connect with an internal ROS 2 network."
    },

    {
        optionIndex::LISTENING_ADDRESSES,
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
        optionIndex::CONNECTION_ADDRESSES,
        0,
        "c",
        "connection-addresses",
        Arg::DSLocator,
        "  -c \t--connection-addresses\t<string>  \t" \
        "List of tuples 'ip,port,server_id' for connection addresses. " \
        "External Participant will use Discovery Server Protocol. In order to connect with remote servers" \
        "is necessary to set an external connection address or use this DataBroker listening address as" \
        "remote connection of other DataBroker. " \
        "Format: list of tuples 'ip,port,server_id' (separated by ',') separated by ';'. " \
        "Default: none"
    },

    {
        optionIndex::INTERACTIVE,
        0,
        "i",
        "interactive",
        Arg::Optional,
        "  -i \t--interactive\t  \t" \
        "Use DataBroker in interactive mode. " \
        "Interactive mode keeps DataBroker listening for user commands. " \
        "These commands are 'help', 'add <topic>', 'remove <topic>', 'exit'."
    },

    { 0, 0, 0, 0, 0, 0 }
};

} /* namespace databroker */
} /* namespace eprosima */

#endif /* EPROSIMA_DATABROKER_ARG_CONFIGURATION_HPP */
