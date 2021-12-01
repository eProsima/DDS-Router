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
 * @file configuration_implementations.cpp
 *
 */

#include <ddsrouter/exceptions/ConfigurationException.hpp>
#include <ddsrouter/exceptions/UnsupportedException.hpp>
#include <ddsrouter/types/address/Address.hpp>
#include <ddsrouter/types/address/DiscoveryServerConnectionAddress.hpp>
#include <ddsrouter/types/configuration_tags.hpp>
#include <ddsrouter/types/endpoint/DomainId.hpp>
#include <ddsrouter/types/endpoint/GuidPrefix.hpp>
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
            ip_ = configuration.as<IpType>();
        }
        catch(const std::exception& e)
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
            port_ = configuration.as<PortType>();
        }
        catch(const std::exception& e)
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
            std::string tag = configuration.as<std::string>();

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
                    "Error getting Address transport type: it must be set <udp> or <tcp>");
            }
        }
        catch(const std::exception& e)
        {
            throw ConfigurationException(utils::Formatter() <<
                "Error getting Address transport type: " << e.what());
        }
    }
    else
    {
        transport_protocol_ = default_transport;
    }

    // TODO do ip version
}

RawConfiguration Address::dump(RawConfiguration&) const
{
    // TODO
    throw UnsupportedException("Address::dump is not supported yet.");
}

DomainId::DomainId(
    const RawConfiguration& configuration)
{
    if (configuration[DOMAIN_ID_TAG])
    {
        // Try to get domain value from configuration
        try
        {
            domain_id_ = configuration[DOMAIN_ID_TAG].as<DomainIdType>();
        }
        catch (const std::exception& e)
        {
            throw ConfigurationException(utils::Formatter() <<
                "Domain has incorrect format" << e.what());
        }
    }
    else
    {
        throw ConfigurationException(
            "Not domain tag found");
    }
}

RawConfiguration DomainId::dump(RawConfiguration&) const
{
    // TODO
    throw UnsupportedException("DomainId::dump is not supported yet.");
}

GuidPrefix::GuidPrefix(
        const RawConfiguration& configuration)
{
    // Check if tag for guid exists
    if (configuration[DISCOVERY_SERVER_GUID_TAG])
    {
        // Try to get domain value from configuration
        try
        {
            *this =
                GuidPrefix(configuration[DISCOVERY_SERVER_GUID_TAG].as<std::string>());
        }
        catch (const std::exception& e)
        {
            throw ConfigurationException(utils::Formatter() <<
                "Guid Prefix has incorrect format" << e.what());
        }
    }

    // Check if tag for Ros exists
    bool ros_guid = false;
    if (configuration[DISCOVERY_SERVER_ID_ROS_TAG])
    {
        // Try to get domain value from configuration
        try
        {
            ros_guid = configuration[DISCOVERY_SERVER_ID_ROS_TAG].as<bool>();
        }
        catch (const std::exception& e)
        {
            throw ConfigurationException(utils::Formatter() <<
                "Ros Guid Prefix Tag has incorrect format. Must be boolean." << e.what());
        }
    }

    // Check if id exists, if not use default
    bool has_id = false;
    uint32_t id;
    if (configuration[DISCOVERY_SERVER_ID_TAG])
    {
        // Try to get domain value from configuration
        try
        {
            id = configuration[DISCOVERY_SERVER_ID_TAG].as<uint32_t>();
            has_id = true;
        }
        catch (const std::exception& e)
        {
            throw ConfigurationException(utils::Formatter() <<
                "Guid Prefix Id has incorrect format. Must be uint32." << e.what());
        }
    }

    if (has_id)
    {
        *this = GuidPrefix(ros_guid, id);
    }
    else
    {
        *this = GuidPrefix(ros_guid);
    }
}

RawConfiguration GuidPrefix::dump(RawConfiguration&) const
{
    // TODO
    throw UnsupportedException("GuidPrefix::dump is not supported yet.");
}

DiscoveryServerConnectionAddress::DiscoveryServerConnectionAddress(
        const RawConfiguration& configuration,
        TransportProtocol default_transport /*= Address::default_transport_protocol()*/)
{
    // Get DS guid. If it fails, throw exception forward
    discovery_server_guid_ = GuidPrefix(configuration);

    // Get Addresses from a list
    if (configuration[COLLECTION_ADDRESSES_TAG])
    {
        // It must be a sequence
        if (!configuration[COLLECTION_ADDRESSES_TAG].IsSequence())
        {
            throw ConfigurationException(
                utils::Formatter() <<
                "Discovery Server Connection Address must have sequence of address under tag: " <<
                COLLECTION_ADDRESSES_TAG);
        }

        // For each element in sequence, get address
        for (auto address : configuration[COLLECTION_ADDRESSES_TAG])
        {
            // In case some of it fails, throw exception forward
            addresses_.insert(Address(address, default_transport));
        }
    }
}

RawConfiguration DiscoveryServerConnectionAddress::dump(RawConfiguration&) const
{
    // TODO
    throw UnsupportedException("DiscoveryServerConnectionAddress::dump is not supported yet.");
}

} /* namespace ddsrouter */
} /* namespace eprosima */
