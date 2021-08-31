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
 * @file DataBrokerConfiguration.cpp
 *
 */

#include <fstream>
#include <yaml-cpp/yaml.h>

#include <fastdds/dds/log/Log.hpp>

#include <databroker/DataBrokerConfiguration.hpp>
#include <databroker/Address.hpp>
#include <databroker/utils.hpp>

namespace eprosima {
namespace databroker {

bool DataBrokerConfiguration::load_default_configuration(
        DataBrokerConfiguration& configuration)
{
    configuration.listening_addresses.push_back(Address("127.0.0.1,11800"));
    configuration.seconds = 0;
    configuration.interactive = false;
    configuration.ros = false;
    configuration.domain = 0;
    configuration.udp = false;
    configuration.server_guid = Address::guid_server();

    load_configuration_file(configuration);

    return true;
}

bool DataBrokerConfiguration::load_configuration_file(
        DataBrokerConfiguration& configuration,
        const std::string& file_path /* ="DATABROKER_CONFIGURATION.yaml" */,
        bool verbose /* = false */)
{
    YAML::Node config_node;
    try
    {
        config_node = YAML::LoadFile(file_path);

        logInfo(DATABROKER_CONFIGURATION, "Loaded file " << file_path);

        // Server ID
        if (config_node["server-id"])
        {
            configuration.server_guid = Address::guid_server(config_node["server-id"].as<int>());
            logInfo(DATABROKER_CONFIGURATION, "Server GUID set by id: " << configuration.server_guid);
        }

        // Server GUID
        if (config_node["server-guid"])
        {
            configuration.server_guid = Address::guid_server(config_node["server-guid"].as<std::string>());
            logInfo(DATABROKER_CONFIGURATION, "Server GUID set: " << configuration.server_guid);
        }

        // Listening address
        if (config_node["listening-addresses"])
        {
            configuration.listening_addresses.clear();
            for (auto address : config_node["listening-addresses"])
            {
                Address new_address;
                if (address["ip"])
                {
                    new_address.ip = address["ip"].as<std::string>();
                }
                if (address["port"])
                {
                    new_address.port = address["port"].as<uint32_t>();
                }
                configuration.listening_addresses.push_back(new_address);
                logInfo(DATABROKER_CONFIGURATION, "Adding address to listening addresses: " << new_address);
            }
        }

        // Connection address
        if (config_node["connection-addresses"])
        {
            configuration.connection_addresses.clear();
            for (auto address : config_node["connection-addresses"])
            {
                Address new_address;
                if (address["ip"])
                {
                    new_address.ip = address["ip"].as<std::string>();
                }
                if (address["port"])
                {
                    new_address.port = address["port"].as<uint32_t>();
                }
                if (address["id"])
                {
                    new_address.guid = Address::guid_server(address["id"].as<uint16_t>());
                }
                if (address["guid"])
                {
                    new_address.guid = Address::guid_server(address["guid"].as<std::string>());
                }
                configuration.connection_addresses.push_back(new_address);
                logInfo(DATABROKER_CONFIGURATION, "Adding address to connection addresses: " << new_address);
            }
        }

        // Whitelist
        if (config_node["whitelist"])
        {
            configuration.active_topics.clear();
            for (auto topic : config_node["whitelist"])
            {
                configuration.active_topics.push_back(topic.as<std::string>());
                logInfo(DATABROKER_CONFIGURATION, "Adding topic to whitelist: " << topic.as<std::string>());
            }
        }

        // ROS
        if (config_node["ros"])
        {
            configuration.ros = config_node["ros"].as<bool>();
            if (configuration.ros)
            {
                logInfo(DATABROKER_CONFIGURATION, "Connecting to a local ROS2 network");
            }
        }

        // UDP
        if (config_node["udp"])
        {
            configuration.udp = config_node["udp"].as<bool>();
            if (configuration.ros)
            {
                logInfo(DATABROKER_CONFIGURATION, "Using UDP transport");
            }
        }

        // Interactive
        if (config_node["interactive"])
        {
            configuration.interactive = config_node["interactive"].as<bool>();
            if (configuration.ros)
            {
                logInfo(DATABROKER_CONFIGURATION, "Set interactive mode");
            }
        }
    }
    catch (const std::exception& e)
    {
        if (verbose)
        {
            logError(DATABROKER_CONFIGURATION, e.what());
        }
        else
        {
            logInfo(DATABROKER, e.what());
        }
        return false;
    }

    return true;
}

} /* namespace databroker */
} /* namespace eprosima */
