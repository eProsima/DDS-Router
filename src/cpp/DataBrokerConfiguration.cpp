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
#include <fastdds/rtps/attributes/RTPSParticipantAttributes.h>

#include <databroker/DataBrokerConfiguration.hpp>
#include <databroker/Address.hpp>
#include <databroker/utils.hpp>

namespace eprosima {
namespace databroker {

bool DataBrokerConfiguration::load_default_configuration(
        DataBrokerConfiguration& configuration)
{
    // DataBroker configuration
    configuration.seconds = 0;
    configuration.interactive = false;
    configuration.active_topics = std::vector<std::string>();

    // WAN Participant configuration
    configuration.wan_configuration.domain = 0;
    configuration.wan_configuration.server_guid = Address::guid_server();
    configuration.wan_configuration.connection_addresses = std::vector<Address>();
    configuration.wan_configuration.listening_addresses = std::vector<Address>();
    configuration.wan_configuration.listening_addresses.push_back(Address("127.0.0.1,11800"));
    configuration.wan_configuration.udp = false;

    // WAN Participant TLS configuration
    configuration.wan_configuration.tls = false;
    configuration.wan_configuration.tls_private_key = DEFAULT_PRIVATE_KEY_FILE;
    configuration.wan_configuration.tls_password = "password";
    configuration.wan_configuration.tls_dh_params = DEFAULT_DH_PARAMS_FILE;
    configuration.wan_configuration.tls_ca_cert = DEFAULT_CA_CERTIFICATE_FILE;
    configuration.wan_configuration.tls_cert = DEFAULT_CERTIFICATE_FILE;

    // Local Participant configuration
    configuration.local_configuration.domain = 0;
    configuration.local_configuration.ros = false;
    configuration.local_configuration.discovery_server = false;
    configuration.local_configuration.discovery_protocol =
            eprosima::fastrtps::rtps::DiscoveryProtocol::SIMPLE;
    configuration.local_configuration.listening_addresses = std::vector<Address>();
    configuration.local_configuration.server_guid =
            eprosima::fastrtps::rtps::GUID_t::unknown().guidPrefix;

    return true;
}

bool DataBrokerConfiguration::load_configuration_file(
        DataBrokerConfiguration& configuration,
        const std::string& file_path /* ="DATABROKER_CONFIGURATION.yaml" */,
        bool verbose /* = false */)
{
    // TODO add log comments
    YAML::Node config_node;

    logInfo(DATABROKER_CONFIGURATION, "Loading configuration file '" << file_path << "'");

    // Whenever the configuration file option is set, the default file path is changed. This is because, even
    // if it fails the first time to load the file, the user might change the file and want to reload it.
    configuration.config_file = file_path;

    try
    {
        config_node = YAML::LoadFile(file_path);

        // Server ID
        if (config_node["server-id"])
        {
            configuration.wan_configuration.server_guid = Address::guid_server(config_node["server-id"].as<int>());
            logInfo(DATABROKER_CONFIGURATION, "Server GUID set by id: " << configuration.wan_configuration.server_guid);
        }

        // Server GUID
        if (config_node["server-guid"])
        {
            configuration.wan_configuration.server_guid = Address::guid_server(
                config_node["server-guid"].as<std::string>());
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
            if (configuration.wan_configuration.udp)
            {
                logInfo(DATABROKER_CONFIGURATION, "Using UDP transport");
            }
        }

        // TLS
        if (config_node["tls"])
        {
            // In case any field is missing, the defaukt configuration will be used
            if (config_node["tls"]["private_key"])
            {
                configuration.wan_configuration.tls_private_key = config_node["tls"]["private_key"].as<std::string>();
            }
            else
            {
                logError(DATABROKER_CONFIGURATION, "TLS configuration needs a private key file");
                return false;
            }

            if (config_node["tls"]["password"])
            {
                configuration.wan_configuration.tls_password = config_node["tls"]["password"].as<std::string>();
            }
            else
            {
                configuration.wan_configuration.tls_password = "";
            }

            if (config_node["tls"]["dh_params"])
            {
                configuration.wan_configuration.tls_dh_params = config_node["tls"]["dh_params"].as<std::string>();
            }
            else
            {
                configuration.wan_configuration.tls_dh_params = "";
            }

            if (config_node["tls"]["ca_cert"])
            {
                configuration.wan_configuration.tls_ca_cert = config_node["tls"]["ca_cert"].as<std::string>();
            }
            else
            {
                logError(DATABROKER_CONFIGURATION, "TLS configuration needs a verify CA certificate file");
                return false;
            }

            if (config_node["tls"]["cert"])
            {
                configuration.wan_configuration.tls_cert = config_node["tls"]["cert"].as<std::string>();
            }
            else
            {
                logError(DATABROKER_CONFIGURATION, "TLS configuration needs a certificate file");
                return false;
            }


            configuration.wan_configuration.tls = true;

            logInfo(DATABROKER_CONFIGURATION, "Using TLS security");
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

        // time
        if (config_node["time"])
        {
            configuration.seconds = config_node["time"].as<uint32_t>();
            logInfo(DATABROKER_CONFIGURATION, "Set time to " << configuration.seconds << " seconds");
        }

        // Domain
        if (config_node["domain"])
        {
            configuration.local_configuration.domain = config_node["domain"].as<uint32_t>();
            logInfo(DATABROKER_CONFIGURATION, "Using internal domain " << configuration.local_configuration.domain);
        }

        // LOCAL DISCOVERY
        if (config_node["local-discovery"])
        {
            if (config_node["local-discovery"]["discovery-server"])
            {
                configuration.local_configuration.discovery_server =
                        config_node["local-discovery"]["discovery-server"].as<bool>();
                if (configuration.local_configuration.discovery_server)
                {
                    configuration.local_configuration.discovery_protocol =
                            eprosima::fastrtps::rtps::DiscoveryProtocol::SERVER;

                    configuration.local_configuration.listening_addresses.clear();
                    bool empty_listening_addresses = true;
                    for (auto address : config_node["local-discovery"]["listening-addresses"])
                    {
                        empty_listening_addresses = false;
                        Address new_address;
                        if (address["ip"])
                        {
                            new_address.ip = address["ip"].as<std::string>();
                        }
                        if (address["port"])
                        {
                            new_address.port = address["port"].as<uint32_t>();
                        }
                        configuration.local_configuration.listening_addresses.push_back(new_address);
                    }

                    if (empty_listening_addresses)
                    {
                        logError(DATABROKER_CONFIGURATION, "No listening addresses set for local discovery server")
                        return false;
                    }

                    configuration.local_configuration.server_guid =
                            Address::ros_discovery_server_guid();
                }
            }
        }

        logInfo(DATABROKER_CONFIGURATION, "Loaded configuration file '" << file_path << "'");
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

bool DataBrokerConfiguration::reload_configuration_file(
        DataBrokerConfiguration& configuration,
        const std::string& file_path /* = "" */)
{
    YAML::Node config_node;

    if (!file_path.empty())
    {
        configuration.config_file = file_path;
    }

    logInfo(DATABROKER_CONFIGURATION, "Reloading configuration file '" << configuration.config_file.c_str() << "'");

    try
    {
        config_node = YAML::LoadFile(configuration.config_file);

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
    }
    catch (const std::exception& e)
    {
        logWarning(DATABROKER_CONFIGURATION, e.what());
        return false;
    }

    return true;
}

} /* namespace databroker */
} /* namespace eprosima */
