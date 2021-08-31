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
    configuration.seconds = 0;
    configuration.interactive = false;
    // configuration.active_topics = std::vector<std::string>();
    configuration.wan_configuration.domain = 0;
    configuration.wan_configuration.server_guid = Address::guid_server();
    // configuration.wan_configuration.connection_addresses = std::vector<Address>();
    // configuration.wan_configuration.listening_addresses = std::vector<Address>();
    configuration.wan_configuration.listening_addresses.push_back(Address("127.0.0.1,11800"));
    configuration.wan_configuration.udp = false;
    configuration.wan_configuration.tls = false;

    configuration.wan_configuration.tls_private_key = "pk.pem";
    configuration.wan_configuration.tls_password = "password";
    configuration.wan_configuration.tls_dh = "dh.pem";
    configuration.wan_configuration.tls_ca = "ca.pem";
    configuration.wan_configuration.tls_verify_ca = "ca.pem";

    configuration.local_configuration.domain = 0;
    configuration.local_configuration.ros = false;

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
            configuration.wan_configuration.server_guid = Address::guid_server(config_node["server-id"].as<int>());
            logInfo(DATABROKER_CONFIGURATION, "Server GUID set by id: " << configuration.wan_configuration.server_guid);
        }

        // Server GUID
        if (config_node["server-guid"])
        {
            configuration.wan_configuration.server_guid = Address::guid_server(config_node["server-guid"].as<std::string>());
            logInfo(DATABROKER_CONFIGURATION, "Server GUID set: " << configuration.wan_configuration.server_guid);
        }

        // Listening address
        if (config_node["listening-addresses"])
        {
            configuration.wan_configuration.listening_addresses.clear();
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
                configuration.wan_configuration.listening_addresses.push_back(new_address);
                logInfo(DATABROKER_CONFIGURATION, "Adding address to listening addresses: " << new_address);
            }
        }

        // Connection address
        if (config_node["connection-addresses"])
        {
            configuration.wan_configuration.connection_addresses.clear();
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
                configuration.wan_configuration.connection_addresses.push_back(new_address);
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
            configuration.local_configuration.ros = config_node["ros"].as<bool>();
            if (configuration.local_configuration.ros)
            {
                logInfo(DATABROKER_CONFIGURATION, "Connecting to a local ROS2 network");
            }
        }

        // UDP
        if (config_node["udp"])
        {
            configuration.wan_configuration.udp = config_node["udp"].as<bool>();
            if (configuration.local_configuration.ros)
            {
                logInfo(DATABROKER_CONFIGURATION, "Using UDP transport");
            }
        }

        // TLS
        if (config_node["tls"])
        {
            configuration.wan_configuration.tls = config_node["tls"].as<bool>();
            if (configuration.local_configuration.ros)
            {
                logInfo(DATABROKER_CONFIGURATION, "Using TLS security");
            }
        }

        // Interactive
        if (config_node["interactive"])
        {
            configuration.interactive = config_node["interactive"].as<bool>();
            if (configuration.local_configuration.ros)
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
