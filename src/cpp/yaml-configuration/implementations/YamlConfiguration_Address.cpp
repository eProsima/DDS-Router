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
 * @file Address_configuration.cpp
 *
 */

#include <ddsrouter/exception/ConfigurationException.hpp>
#include <ddsrouter/types/address/Address.hpp>
#include <ddsrouter/types/address/DiscoveryServerConnectionAddress.hpp>
#include <ddsrouter/yaml-configuration/YamlConfiguration.hpp>
#include <ddsrouter/yaml-configuration/yaml_configuration_tags.hpp>

namespace eprosima {
namespace ddsrouter {
namespace yaml {

Address YamlElementConfiguration::address(const Yaml& yaml)
{
    IpType ip = Address::default_ip();
    PortType port = Address::default_port();
    IpVersion ip_version = Address::default_ip_version();
    TransportProtocol transport_protocol = Address::default_transport_protocol();

    // TODO
    // may be removed when schema works
    if (!yaml.IsMap() && !yaml.IsNull())
    {
        throw ConfigurationException("Address expects a map as base yaml type or an empty one.");
    }

    ////////////////
    // IP VERSION

    // Variable to store that ip version has been manually set by user, or it should be derived from ip
    bool ip_version_has_been_set = true;

    // Get IP version. This needs to go first because IP and DNS needs to know the IP version
    if (yaml[ADDRESS_IP_VERSION_TAG])
    {
        try
        {
            std::string tag = yaml[ADDRESS_IP_VERSION_TAG].as<std::string>();

            if (tag == ADDRESS_IP_VERSION_V4_TAG)
            {
                ip_version = IPv4;
            }
            else if (tag == ADDRESS_IP_VERSION_V6_TAG)
            {
                ip_version = IPv6;
            }
            else
            {
                throw ConfigurationException(utils::Formatter() <<
                              "Error getting Address ip version: it must be <v4> or <v6>");
            }
        }
        catch (const std::exception& e)
        {
            throw ConfigurationException(utils::Formatter() <<
                          "Error getting Address ip version: " << e.what());
        }
    }
    else
    {
        // Set ip version default
        ip_version_has_been_set = false;
    }

    ////////////////
    // IP

    // Variable to store that ip has been manually set by user, or it should be default
    bool ip_has_been_set = true;

    // Get IP
    if (yaml[ADDRESS_IP_TAG])
    {
        try
        {
            ip = yaml[ADDRESS_IP_TAG].as<IpType>();

            if (Address::is_ipv4_correct(ip))
            {
                if (!ip_version_has_been_set)
                {
                    ip_version = IPv4;
                    ip_version_has_been_set = true;
                }
                else
                {
                    // In case it is ipv4 but was specified ipv6, not fail and show warning
                    if (ip_version != IPv4)
                    {
                        logWarning(
                            DDSROUTER_ADDRESS_CONFIGURATION,
                            "Using ip " << ip << " as IPv4 even if it was specified to be IPv6."; );
                        ip_version = IPv4;
                    }
                }
            }
            else if (Address::is_ipv6_correct(ip))
            {
                if (!ip_version_has_been_set)
                {
                    ip_version = IPv6;
                    ip_version_has_been_set = true;
                }
                else
                {
                    // In case it is ipv6 but was specified ipv4, not fail and show warning
                    if (ip_version != IPv6)
                    {
                        logWarning(
                            DDSROUTER_ADDRESS_CONFIGURATION,
                            "Using ip " << ip << " as IPv6 even if it was specified to be IPv4."; );
                        ip_version = IPv6;
                    }
                }
            }
            else
            {
                throw ConfigurationException(utils::Formatter() <<
                              "Incorrect IP Address: " << ip);
            }
        }
        catch (const std::exception& e)
        {
            throw ConfigurationException(utils::Formatter() <<
                          "Error getting Address ip: " << e.what());
        }
    }
    else
    {
        ip_has_been_set = false;
    }


    ////////////////
    // DNS

    // Get IP from Domain Name calling DNS
    if (yaml[ADDRESS_DNS_TAG])
    {
        // If IP was specified, do not get domain
        if (ip_has_been_set)
        {
            logWarning(
                DDSROUTER_ADDRESS_CONFIGURATION,
                "Ip was specified, domain tag skipped.");
        }
        else
        {
            try
            {
                std::string domain_name = yaml[ADDRESS_DNS_TAG].as<std::string>();

                // Call DNS
                auto response = fastrtps::rtps::IPLocator::resolveNameDNS(domain_name);

                if (ip_version_has_been_set)
                {
                    // Add the first valid IPv6 found
                    if (ip_version == IPv6)
                    {
                        if (response.second.size() > 0)
                        {
                            ip = response.second.begin()->data();
                            ip_has_been_set = true;
                        }
                        else
                        {
                            throw ConfigurationException(utils::Formatter() <<
                                          "DNS not found for IPv6 with domain: " << domain_name);
                        }
                    }
                    else
                    {
                        // Getting IPv4
                        if (response.first.size() > 0)
                        {
                            ip = response.first.begin()->data();
                            ip_has_been_set = true;
                        }
                        else
                        {
                            throw ConfigurationException(utils::Formatter() <<
                                          "DNS not found for IPv4 with domain: " << domain_name);
                        }
                    }
                }
                else
                {
                    // Ip version has not beed specified, so use IPv4 as default
                    // If IPv4 has no results, get IPv6
                    if (response.first.size() > 0)
                    {
                        ip = response.first.begin()->data();
                        ip_version = IPv4;

                        ip_version_has_been_set = true;
                        ip_has_been_set = true;
                    }
                    else if (response.second.size() > 0)
                    {
                        ip = response.second.begin()->data();
                        ip_version = IPv6;

                        ip_version_has_been_set = true;
                        ip_has_been_set = true;
                    }
                    else
                    {
                        throw ConfigurationException(utils::Formatter() <<
                                      "DNS address not found: " << domain_name);
                    }
                }
            }
            catch (const std::exception& e)
            {
                throw ConfigurationException(utils::Formatter() <<
                              "Error getting Domain Name: " << e.what());
            }
        }
    }

    ////////////////
    // GETTING DEFAULT IP

    // If ip has not been set, use default ip
    if (!ip_has_been_set)
    {
        // If ip version has not been set, use default version
        if (!ip_version_has_been_set)
        {
            ip_version = Address::default_ip_version();
        }
        ip = Address::default_ip(ip_version);
    }

    ////////////////
    // PORT

    // Get Port
    if (yaml[ADDRESS_PORT_TAG])
    {
        try
        {
            port = yaml[ADDRESS_PORT_TAG].as<PortType>();
        }
        catch (const std::exception& e)
        {
            throw ConfigurationException(utils::Formatter() <<
                          "Error getting Address port: " << e.what());
        }
    }

    ////////////////
    // TRANSPORT

    // Get transport
    if (yaml[ADDRESS_TRANSPORT_TAG])
    {
        try
        {
            std::string tag = yaml[ADDRESS_TRANSPORT_TAG].as<std::string>();

            if (tag == ADDRESS_TRANSPORT_UDP_TAG)
            {
                transport_protocol = UDP;
            }
            else if (tag == ADDRESS_TRANSPORT_TCP_TAG)
            {
                transport_protocol = TCP;
            }
            else
            {
                throw ConfigurationException(utils::Formatter() <<
                              "Error getting Address transport type: it must be <udp> or <tcp>");
            }
        }
        catch (const std::exception& e)
        {
            throw ConfigurationException(utils::Formatter() <<
                          "Error getting Address transport type: " << e.what());
        }
    }

    return Address(ip, port, ip_version, transport_protocol);
}

DiscoveryServerConnectionAddress YamlElementConfiguration::discovery_server_connection_address(const Yaml& yaml)
{
    GuidPrefix discovery_server_guid;
    std::set<Address> addresses;

    // Get DS guid. If it fails, throw exception forward
    discovery_server_guid = GuidPrefix(yaml);

    // Get Discovery Server Guid Prefix
    try
    {

        if (yaml[DISCOVERY_SERVER_GUID_PREFIX_TAG])
        {
            discovery_server_guid = guid_prefix(yaml[DISCOVERY_SERVER_GUID_PREFIX_TAG]);
        }
        else
        {
            throw ConfigurationException(
                    "Discovery Server connection-address requires a Guid Prefix specification.");
        }
    }
    catch(const std::exception& e)
    {
        throw ConfigurationException(utils::Formatter() <<
                "Error getting Discovery Server Guid Prefix in Connection Address: " << e.what());
    }

    // Get addresses
    try
    {
        // Get Addresses from a list
        if (yaml[COLLECTION_ADDRESSES_TAG])
        {
            // It must be a sequence
            if (!yaml[COLLECTION_ADDRESSES_TAG].IsSequence())
            {
                throw ConfigurationException(
                        utils::Formatter() <<
                            "Discovery Server Connection Address must have sequence of address under tag: " <<
                            COLLECTION_ADDRESSES_TAG);
            }

            // For each element in sequence, get address
            for (auto address_yaml : yaml[COLLECTION_ADDRESSES_TAG])
            {
                // In case some of it fails, throw exception forward
                addresses.insert(address(address_yaml));
            }
        }
    }
    catch(const std::exception& e)
    {
        throw ConfigurationException(utils::Formatter() <<
                "Error getting Discovery Server Connection Addresses: " << e.what());
    }

    return DiscoveryServerConnectionAddress(discovery_server_guid, addresses);
}

} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */
