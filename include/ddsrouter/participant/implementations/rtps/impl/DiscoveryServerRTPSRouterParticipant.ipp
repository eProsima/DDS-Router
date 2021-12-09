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
 * @file DiscoveryServerRTPSRouterParticipant.cpp
 */

#ifndef _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_AUX_DISCOVERYSERVERRTPSROUTERPARTICIPANT_IMPL_IPP_
#define _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_AUX_DISCOVERYSERVERRTPSROUTERPARTICIPANT_IMPL_IPP_

#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/TCPv4TransportDescriptor.h>

#include <ddsrouter/participant/implementations/rtps/DiscoveryServerRTPSRouterParticipant.hpp>

namespace eprosima {
namespace ddsrouter {
namespace rtps {


template <class ConfigurationType>
DiscoveryServerRTPSRouterParticipant<ConfigurationType>::DiscoveryServerRTPSRouterParticipant(
        const ParticipantConfiguration& participant_configuration,
        std::shared_ptr<PayloadPool> payload_pool,
        std::shared_ptr<DiscoveryDatabase> discovery_database)
    : CommonRTPSRouterParticipant<ConfigurationType>
        (participant_configuration, payload_pool, discovery_database)
{
}

template <class ConfigurationType>
fastrtps::rtps::RTPSParticipantAttributes
    DiscoveryServerRTPSRouterParticipant<ConfigurationType>::participant_attributes_() const noexcept
{
    // Get Configuration information
    std::vector<Address> listening_addresses = this->configuration_.listening_addresses();
    std::vector<DiscoveryServerConnectionAddress> connection_addresses = this->configuration_.connection_addresses();
    GuidPrefix discovery_server_guid = this->configuration_.discovery_server_guid();

    // Set attributes
    fastrtps::rtps::RTPSParticipantAttributes params;
        // CommonRTPSRouterParticipant::participant_attributes(); // Use default as base attributes

    // Needed values to check at the end if descriptor must be set
    bool has_listening_addresses = false;
    bool has_connection_addresses = false;
    bool has_listening_tcp = false;
    bool has_connection_tcp = false;
    bool has_udp = false;

    params.useBuiltinTransports = false;

    /////
    // Set listening addresses
    for (Address address : listening_addresses)
    {
        if (!address.is_valid())
        {
            // Invalid address, continue with next one
            logInfo(DDSROUTER_DISCOVERYSERVER_PARTICIPANT,
                "Discard listening address: " << address << " in Participant " << this->id() << " initialization.");
            continue;
        }

        has_listening_addresses = true;

        // TCP Listening WAN address
        if (address.is_tcp())
        {
            has_listening_tcp = true;

            std::shared_ptr<eprosima::fastdds::rtps::TCPv4TransportDescriptor> descriptor =
                    std::make_shared<eprosima::fastdds::rtps::TCPv4TransportDescriptor>();

            descriptor->add_listener_port(address.port());
            descriptor->set_WAN_address(address.ip());

            descriptor->sendBufferSize = 0;
            descriptor->receiveBufferSize = 0;

            // TODO enable TLS

            params.userTransports.push_back(descriptor);
        }
        else
        {
            has_udp = true;
        }

        // For any, UDP or TCP
        // Create Locator
        eprosima::fastrtps::rtps::Locator_t locator;
        locator.kind = address.get_locator_kind();

        // IP
        if (address.is_ipv4())
        {
            eprosima::fastrtps::rtps::IPLocator::setIPv4(locator, address.ip());
            eprosima::fastrtps::rtps::IPLocator::setWan(locator, address.ip());
        }
        else
        {
            eprosima::fastrtps::rtps::IPLocator::setIPv6(locator, address.ip());
        }

        // Port
        eprosima::fastrtps::rtps::IPLocator::setPhysicalPort(locator, address.port());
        eprosima::fastrtps::rtps::IPLocator::setLogicalPort(locator, address.port());

        // Add listening address to builtin
        params.builtin.metatrafficUnicastLocatorList.push_back(locator);

        logDebug(DDSROUTER_DISCOVERYSERVER_PARTICIPANT,
            "Add listening address " << address << " to Participant " << this->id() << ".");
    }

    /////
    // Set connection addresses
    for (DiscoveryServerConnectionAddress connection_address : connection_addresses)
    {
        if (!connection_address.is_valid())
        {
            // Invalid address, continue with next one
            logInfo(DDSROUTER_DISCOVERYSERVER_PARTICIPANT,
                "Discard connection address with remote server: " << connection_address.discovery_server_guid() <<
                " in Participant " << this->id() << " initialization.");
            continue;
        }

        // Set Server GUID
        GuidPrefix server_prefix = connection_address.discovery_server_guid();

        for (Address address : connection_address.addresses())
        {
            has_connection_addresses = true;

            eprosima::fastrtps::rtps::RemoteServerAttributes server_attr;
            server_attr.guidPrefix = server_prefix;

            eprosima::fastrtps::rtps::Locator_t locator;

            // KIND
            locator.kind = address.get_locator_kind();

            // In case it is TCP mark has_connection_tcp as true
            if (address.is_tcp())
            {
                has_connection_tcp = true;
            }
            else
            {
                has_udp = true;
            }

            // IP
            if (address.is_ipv4())
            {
                eprosima::fastrtps::rtps::IPLocator::setIPv4(locator, address.ip());
            }
            else
            {
                eprosima::fastrtps::rtps::IPLocator::setIPv6(locator, address.ip());
            }

            // PORT
            eprosima::fastrtps::rtps::IPLocator::setPhysicalPort(locator, address.port());
            eprosima::fastrtps::rtps::IPLocator::setLogicalPort(locator, address.port());
            // Warning: Logical port is not needed unless domain could change

            // Add as remote server and add it to builtin
            server_attr.metatrafficUnicastLocatorList.push_back(locator);
            params.builtin.discovery_config.m_DiscoveryServers.push_back(server_attr);

            logDebug(DDSROUTER_DISCOVERYSERVER_PARTICIPANT,
                "Add connection address " << address << " for server " << server_prefix <<
                " to Participant " << this->id() << ".");
        }
    }

    /////
    // Set this participant as a SERVER if has listening locators
    if (has_listening_addresses)
    {
        params.builtin.discovery_config.discoveryProtocol =
                fastrtps::rtps::DiscoveryProtocol::SERVER;
    }
    else
    {
        params.builtin.discovery_config.discoveryProtocol =
                fastrtps::rtps::DiscoveryProtocol::SUPER_CLIENT;

        if (!has_connection_addresses)
        {
            logWarning(DDSROUTER_DISCOVERYSERVER_PARTICIPANT,
                "Creating Participant " << this->id() << " without listening or connection addresses. " <<
                "It will not communicate with any other Participant.");
        }
    }

    /////
    // Set Server Guid
    params.prefix = this->configuration_.discovery_server_guid();

    /////
    // Create specific descriptors if needed

    // TODO create v6 descriptors

    // If has TCP connections but not TCP listening addresses, it must specify the TCP transport
    if (has_connection_tcp && !has_listening_tcp)
    {
        // TODO enable tls
        std::shared_ptr<eprosima::fastdds::rtps::TCPv4TransportDescriptor> descriptor =
                std::make_shared<eprosima::fastdds::rtps::TCPv4TransportDescriptor>();
        params.userTransports.push_back(descriptor);

        logDebug(DDSROUTER_DISCOVERYSERVER_PARTICIPANT,
            "Adding TCP Transport to Participant " << this->id() << ".");
    }

    // If has UDP, create descriptor because it has not been created yet
    if (has_udp)
    {
        std::shared_ptr<eprosima::fastdds::rtps::UDPv4TransportDescriptor> descriptor =
                std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
        params.userTransports.push_back(descriptor);

        logDebug(DDSROUTER_DISCOVERYSERVER_PARTICIPANT,
            "Adding UDP Transport to Participant " << this->id() << ".");
    }

    logDebug(DDSROUTER_DISCOVERYSERVER_PARTICIPANT,
        "Configured Participant " << this->id() << " with server guid: " << this->configuration_.discovery_server_guid());

    return params;
}

} /* namespace rtps */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_AUX_DISCOVERYSERVERRTPSROUTERPARTICIPANT_IMPL_IPP_ */