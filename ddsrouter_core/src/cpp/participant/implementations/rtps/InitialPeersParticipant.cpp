// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file InitialPeersParticipant.cpp
 */

#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/UDPv6TransportDescriptor.h>
#include <fastdds/rtps/transport/TCPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/TCPv6TransportDescriptor.h>

#include <participant/implementations/rtps/InitialPeersParticipant.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

using namespace eprosima::ddsrouter::core::types;

InitialPeersParticipant::InitialPeersParticipant(
        std::shared_ptr<configuration::InitialPeersParticipantConfiguration> participant_configuration,
        std::shared_ptr<PayloadPool> payload_pool,
        std::shared_ptr<DiscoveryDatabase> discovery_database)
    : CommonParticipant(
        participant_configuration,
        payload_pool,
        discovery_database,
        participant_configuration->domain,
        participant_attributes_(participant_configuration.get()))
{
}

fastrtps::rtps::RTPSParticipantAttributes InitialPeersParticipant::participant_attributes_(
        const configuration::InitialPeersParticipantConfiguration* configuration)
{
    // Use default as base attributes
    fastrtps::rtps::RTPSParticipantAttributes params = CommonParticipant::participant_attributes_(configuration);

    // Auxiliary variable to save characters and improve readability
    const auto& tls_config = configuration->tls_configuration;

    // Needed values to check at the end if descriptor must be set
    bool has_listening_addresses = false;
    bool has_connection_addresses = false;
    bool has_listening_tcp_ipv4 = false;
    bool has_listening_tcp_ipv6 = false;
    bool has_connection_tcp_ipv4 = false;
    bool has_connection_tcp_ipv6 = false;
    bool has_connection_udp_ipv4 = false;
    bool has_connection_udp_ipv6 = false;
    bool has_udp_ipv4 = false;
    bool has_udp_ipv6 = false;

    params.useBuiltinTransports = false;

    /////
    // Set listening addresses
    for (const types::Address& address : configuration->listening_addresses)
    {
        if (!address.is_valid())
        {
            // Invalid address, continue with next one
            logWarning(DDSROUTER_INITIALPEERS_PARTICIPANT,
                    "Discard listening address: " << address <<
                    " in Participant " << configuration->id << " initialization.");
            continue;
        }

        has_listening_addresses = true;

        // TCP Listening WAN address
        if (address.is_tcp())
        {
            if (address.is_ipv4())
            {
                has_listening_tcp_ipv4 = true;

                std::shared_ptr<eprosima::fastdds::rtps::TCPv4TransportDescriptor> descriptor =
                        std::make_shared<eprosima::fastdds::rtps::TCPv4TransportDescriptor>();

                descriptor->add_listener_port(address.port());
                descriptor->set_WAN_address(address.ip());

                // Enable TLS
                if (tls_config.is_active())
                {
                    tls_config.enable_tls(descriptor);
                }

                params.userTransports.push_back(descriptor);
            }
            else
            {
                has_listening_tcp_ipv6 = true;

                std::shared_ptr<eprosima::fastdds::rtps::TCPv6TransportDescriptor> descriptor =
                        std::make_shared<eprosima::fastdds::rtps::TCPv6TransportDescriptor>();

                descriptor->add_listener_port(address.port());

                // Enable TLS
                if (tls_config.is_active())
                {
                    tls_config.enable_tls(descriptor);
                }

                params.userTransports.push_back(descriptor);
            }
        }
        else
        {
            has_udp_ipv4 = address.is_ipv4();
            has_udp_ipv6 = !address.is_ipv4();
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

        // Set Logical port for every locator
        eprosima::fastrtps::rtps::IPLocator::setPhysicalPort(locator, address.port());

        // In TCP case, set Physical port
        if (address.is_tcp())
        {
            eprosima::fastrtps::rtps::IPLocator::setLogicalPort(locator, address.port());
        }

        // Add listening address to builtin
        params.builtin.metatrafficUnicastLocatorList.push_back(locator);
        params.defaultUnicastLocatorList.push_back(locator);

        logDebug(DDSROUTER_INITIALPEERS_PARTICIPANT,
                "Add listening address " << address << " to Participant " << configuration->id << ".");
    }

    /////
    // Set connection addresses
    for (const types::Address& connection_address : configuration->connection_addresses)
    {
        if (!connection_address.is_valid())
        {
            // Invalid connection address, continue with next one
            logWarning(DDSROUTER_INITIALPEERS_PARTICIPANT,
                    "Discard connection address: " << connection_address <<
                    " in Participant " << configuration->id << " initialization.");
            continue;
        }

        has_connection_addresses = true;

        // Create Locator for connection initial peers
        eprosima::fastrtps::rtps::Locator_t locator;

        // KIND
        locator.kind = connection_address.get_locator_kind();

        // In case it is TCP mark has_connection_tcp as true
        if (connection_address.is_tcp())
        {
            has_connection_tcp_ipv4 = connection_address.is_ipv4();
            has_connection_tcp_ipv6 = !connection_address.is_ipv4();
        }
        else
        {
            has_connection_udp_ipv4 = connection_address.is_ipv4();
            has_connection_udp_ipv6 = !connection_address.is_ipv4();
            has_udp_ipv4 = connection_address.is_ipv4();
            has_udp_ipv6 = !connection_address.is_ipv4();
        }

        // IP
        if (connection_address.is_ipv4())
        {
            eprosima::fastrtps::rtps::IPLocator::setIPv4(locator, connection_address.ip());
        }
        else
        {
            eprosima::fastrtps::rtps::IPLocator::setIPv6(locator, connection_address.ip());
        }

        // Set Logical port for every locator
        eprosima::fastrtps::rtps::IPLocator::setPhysicalPort(locator, connection_address.port());

        // In TCP case, set Physical port
        if (connection_address.is_tcp())
        {
            eprosima::fastrtps::rtps::IPLocator::setLogicalPort(locator, connection_address.port());
        }

        // Add it to builtin
        params.builtin.initialPeersList.push_back(locator);

        logDebug(DDSROUTER_INITIALPEERS_PARTICIPANT,
                "Add connection address " << connection_address <<
                " to Participant " << configuration->id << ".");
    }

    /////
    // Create specific descriptors if needed

    // If has TCP connections but not TCP listening addresses, it must specify the TCP transport
    if (has_connection_tcp_ipv4 && !has_listening_tcp_ipv4)
    {
        std::shared_ptr<eprosima::fastdds::rtps::TCPv4TransportDescriptor> descriptor =
                std::make_shared<eprosima::fastdds::rtps::TCPv4TransportDescriptor>();

        // Enable TLS
        if (tls_config.is_active())
        {
            tls_config.enable_tls(descriptor, true);
        }

        params.userTransports.push_back(descriptor);

        logDebug(DDSROUTER_INITIALPEERS_PARTICIPANT,
                "Adding TCPv4 Transport to Participant " << configuration->id << ".");
    }

    if (has_connection_tcp_ipv6 && !has_listening_tcp_ipv6)
    {
        std::shared_ptr<eprosima::fastdds::rtps::TCPv6TransportDescriptor> descriptor =
                std::make_shared<eprosima::fastdds::rtps::TCPv6TransportDescriptor>();

        // Enable TLS
        if (tls_config.is_active())
        {
            tls_config.enable_tls(descriptor, true);
        }

        params.userTransports.push_back(descriptor);

        logDebug(DDSROUTER_INITIALPEERS_PARTICIPANT,
                "Adding TCPv6 Transport to Participant " << configuration->id << ".");
    }

    // If has UDP, create descriptor because it has not been created yet
    if (has_udp_ipv4)
    {
        std::shared_ptr<eprosima::fastdds::rtps::UDPv4TransportDescriptor> descriptor =
                std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
        params.userTransports.push_back(descriptor);

        logDebug(DDSROUTER_INITIALPEERS_PARTICIPANT,
                "Adding UDPv4 Transport to Participant " << configuration->id << ".");
    }

    if (has_udp_ipv6)
    {
        std::shared_ptr<eprosima::fastdds::rtps::UDPv6TransportDescriptor> descriptor_v6 =
                std::make_shared<eprosima::fastdds::rtps::UDPv6TransportDescriptor>();
        params.userTransports.push_back(descriptor_v6);

        logDebug(DDSROUTER_INITIALPEERS_PARTICIPANT,
                "Adding UDPv6 Transport to Participant " << configuration->id << ".");
    }

    // To avoid creating a multicast transport in UDP when non listening addresses
    // Fast requires an empty locator that will be set by default afterwards
    if (!has_listening_addresses)
    {
        if (has_connection_udp_ipv4)
        {
            eprosima::fastrtps::rtps::Locator_t locator;
            locator.kind = LOCATOR_KIND_UDPv4;
            params.builtin.metatrafficUnicastLocatorList.push_back(locator);
        }

        if (has_connection_udp_ipv6)
        {
            eprosima::fastrtps::rtps::Locator_t locator;
            locator.kind = LOCATOR_KIND_UDPv6;
            params.builtin.metatrafficUnicastLocatorList.push_back(locator);
        }
    }

    logDebug(DDSROUTER_INITIALPEERS_PARTICIPANT,
            "Configured Participant " << configuration->id);

    return params;
}

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
