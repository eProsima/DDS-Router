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

#include <ddsrouter/exceptions/ConfigurationException.hpp>
#include <ddsrouter/exceptions/UnsupportedException.hpp>
#include <ddsrouter/types/address/Address.hpp>
#include <ddsrouter/types/configuration_tags.hpp>
#include <ddsrouter/types/RawConfiguration.hpp>

namespace eprosima {
namespace ddsrouter {

Address::Address(
        const RawConfiguration& configuration,
        TransportProtocol default_transport /*= Address::default_transport_protocol()*/)
{
    if (!configuration.IsMap() && !configuration.IsNull())
    {
        throw ConfigurationException("Address expects a map as base yaml type or an empty one.");
    }

    // Get IP version. This needs to go first because the DNS call needs to know the IP version
    if (configuration[ADDRESS_IP_VERSION_TAG])
    {
        try
        {
            std::string tag = configuration[ADDRESS_IP_VERSION_TAG].as<std::string>();

            if (tag == ADDRESS_IP_VERSION_V4_TAG)
            {
                ip_version_ = IPv4;
            }
            else if (tag == ADDRESS_IP_VERSION_V6_TAG)
            {
                ip_version_ = IPv6;
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
        ip_version_ = NONE_IPv;
    }

    // Get IP
    if (configuration[ADDRESS_IP_TAG])
    {
        try
        {
            ip_ = configuration[ADDRESS_IP_TAG].as<IpType>();

            // Check if it is a valid IP or it should call DNS
            if (!Address::is_ipv4_correct(ip_) && !Address::is_ipv6_correct(ip_))
            {
                // Call
                auto response = fastrtps::rtps::IPLocator::resolveNameDNS(ip_);

                // Add the first valid IP found
                if (ip_version_ == IPv6)
                {
                    if (response.second.size() > 0)
                    {
                        ip_ = response.second.begin()->data();
                    }
                    else
                    {
                        throw ConfigurationException(utils::Formatter() <<
                          "Incorrect IPv6 set or DNS not found: " << ip_);
                    }
                }
                else
                {
                    if (response.first.size() > 0)
                    {
                        ip_ = response.first.begin()->data();
                    }
                    else
                    {
                        throw ConfigurationException(utils::Formatter() <<
                          "Incorrect IPv4 set or DNS not found: " << ip_);
                    }
                }
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
        ip_ = Address::default_ip();
    }

    // Check the IP Version is correctly set
    if (ip_version_ == NONE_IPv)
    {
        if (Address::is_ipv4_correct(ip_))
        {
            ip_version_ = IPv4;
        }
        else if (Address::is_ipv6_correct(ip_))
        {
            ip_version_ = IPv6;
        }
        else
        {
            throw ConfigurationException(utils::Formatter() <<
                    "Incorrect IP Address: " << ip_);
        }
    }

    // Get Port
    if (configuration[ADDRESS_PORT_TAG])
    {
        try
        {
            port_ = configuration[ADDRESS_PORT_TAG].as<PortType>();
        }
        catch (const std::exception& e)
        {
            throw ConfigurationException(utils::Formatter() <<
                          "Error getting Address port: " << e.what());
        }
    }
    else
    {
        port_ = Address::default_port();
    }

    // Get transport
    if (configuration[ADDRESS_TRANSPORT_TAG])
    {
        try
        {
            std::string tag = configuration[ADDRESS_TRANSPORT_TAG].as<std::string>();

            if (tag == ADDRESS_TRANSPORT_UDP_TAG)
            {
                transport_protocol_ = UDP;
            }
            else if (tag == ADDRESS_TRANSPORT_TCP_TAG)
            {
                transport_protocol_ = TCP;
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
    else
    {
        transport_protocol_ = default_transport;
    }
}

RawConfiguration Address::dump(
        RawConfiguration&) const
{
    // TODO
    throw UnsupportedException("Address::dump is not supported yet.");
}

} /* namespace ddsrouter */
} /* namespace eprosima */
