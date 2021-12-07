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
    // Get IP
    if (configuration[ADDRESS_IP_TAG])
    {
        try
        {
            ip_ = configuration[ADDRESS_IP_TAG].as<IpType>();
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

    // Get ip version
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
        // Set ip version depending on ip
        if (Address::is_ipv4_correct(ip_))
        {
            ip_version_ = IPv4;
        }
        else
        {
            ip_version_ = IPv6;
        }
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
