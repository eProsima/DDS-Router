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
 * @file DiscoveryServerParticipant.cpp
 */

#ifndef __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_RTPS_DISCOVERYSERVERRTPSROUTERPARTICIPANT_IMPL_IPP_
#define __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_RTPS_DISCOVERYSERVERRTPSROUTERPARTICIPANT_IMPL_IPP_

#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/UDPv6TransportDescriptor.h>
#include <fastdds/rtps/transport/TCPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/TCPv6TransportDescriptor.h>

#include <ddsrouter_utils/exception/ConfigurationException.hpp>
#include <ddsrouter_utils/utils.hpp>

#include <ddsrouter_core/types/security/tls/TlsConfigurationBoth.hpp>

#include <participant/implementations/rtps/DiscoveryServerParticipant.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {


template <class ConfigurationType>
DiscoveryServerParticipant<ConfigurationType>::DiscoveryServerParticipant(
        const ConfigurationType participant_configuration,
        std::shared_ptr<PayloadPool> payload_pool,
        std::shared_ptr<DiscoveryDatabase> discovery_database)
    : CommonRTPSRouterParticipant<ConfigurationType>
        (participant_configuration, payload_pool, discovery_database)
{
}

template <class ConfigurationType>
fastrtps::rtps::RTPSParticipantAttributes
DiscoveryServerParticipant<ConfigurationType>::participant_attributes_() const
{
    // Get Configuration information
    std::set<types::Address> listening_addresses = this->configuration_.listening_addresses();
    std::set<types::DiscoveryServerConnectionAddress> connection_addresses = this->configuration_.connection_addresses();
    types::GuidPrefix discovery_server_guid_prefix = this->configuration_.discovery_server_guid_prefix();
    std::shared_ptr<types::security::TlsConfiguration> tls_config =  this->configuration_.tls_configuration();

    // Set attributes
    fastrtps::rtps::RTPSParticipantAttributes params;
    // CommonRTPSRouterParticipant::participant_attributes(); // Use default as base attributes

    // Needed values to check at the end if descriptor must be set
    bool has_listening_addresses = false;
    bool has_connection_addresses = false;
    bool has_listening_tcp_ipv4 = false;
    bool has_listening_tcp_ipv6 = false;
    bool has_connection_tcp_ipv4 = false;
    bool has_connection_tcp_ipv6 = false;
    bool has_udp_ipv4 = false;
    bool has_udp_ipv6 = false;

    params.useBuiltinTransports = false;

    /////
    // Set listening addresses
    for (types::Address address : listening_addresses)
    {
        if (!address.is_valid())
        {
            // Invalid address, continue with next one
            logWarning(DDSROUTER_DISCOVERYSERVER_PARTICIPANT,
                    "Discard listening address: " << address << " in Participant " << this->id() << " initialization.");
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
                if (tls_config->is_active())
                {
                    enable_tls(descriptor, tls_config);
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
                if (tls_config->is_active())
                {
                    enable_tls(descriptor, tls_config);
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

        // Port
        eprosima::fastrtps::rtps::IPLocator::setPhysicalPort(locator, address.port());
        eprosima::fastrtps::rtps::IPLocator::setLogicalPort(locator, address.port());

        // Add listening address to builtin
        params.builtin.metatrafficUnicastLocatorList.push_back(locator);
        params.defaultUnicastLocatorList.push_back(locator);

        logDebug(DDSROUTER_DISCOVERYSERVER_PARTICIPANT,
                "Add listening address " << address << " to Participant " << this->id() << ".");
    }

    /////
    // Set connection addresses
    for (types::DiscoveryServerConnectionAddress connection_address : connection_addresses)
    {
        if (!connection_address.is_valid())
        {
            // Invalid connection address, continue with next one
            logWarning(DDSROUTER_DISCOVERYSERVER_PARTICIPANT,
                    "Discard connection address with remote server: " <<
                    connection_address.discovery_server_guid_prefix() <<
                    " in Participant " << this->id() << " initialization.");
            continue;
        }

        // Set Server GUID
        types::GuidPrefix server_prefix = connection_address.discovery_server_guid_prefix();

        for (types::Address address : connection_address.addresses())
        {
            if (!address.is_valid())
            {
                // Invalid ip address, continue with next one
                logWarning(DDSROUTER_DISCOVERYSERVER_PARTICIPANT,
                        "Discard connection address with remote server: " <<
                        connection_address.discovery_server_guid_prefix() <<
                        " due to invalid ip address " << address.ip() << " in Participant " << this->id() <<
                        " initialization.");
                continue;
            }

            has_connection_addresses = true;

            eprosima::fastrtps::rtps::RemoteServerAttributes server_attr;
            server_attr.guidPrefix = server_prefix;

            eprosima::fastrtps::rtps::Locator_t locator;

            // KIND
            locator.kind = address.get_locator_kind();

            // In case it is TCP mark has_connection_tcp as true
            if (address.is_tcp())
            {
                has_connection_tcp_ipv4 = address.is_ipv4();
                has_connection_tcp_ipv6 = !address.is_ipv4();
            }
            else
            {
                has_udp_ipv4 = address.is_ipv4();
                has_udp_ipv6 = !address.is_ipv4();
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
    params.prefix = discovery_server_guid_prefix;

    /////
    // Create specific descriptors if needed

    // If has TCP connections but not TCP listening addresses, it must specify the TCP transport
    if (has_connection_tcp_ipv4 && !has_listening_tcp_ipv4)
    {
        std::shared_ptr<eprosima::fastdds::rtps::TCPv4TransportDescriptor> descriptor =
                std::make_shared<eprosima::fastdds::rtps::TCPv4TransportDescriptor>();

        // Enable TLS
        if (tls_config->is_active())
        {
            enable_tls(descriptor, tls_config, true);
        }

        params.userTransports.push_back(descriptor);

        logDebug(DDSROUTER_DISCOVERYSERVER_PARTICIPANT,
                "Adding TCPv4 Transport to Participant " << this->id() << ".");
    }
    if (has_connection_tcp_ipv6 && !has_listening_tcp_ipv6)
    {
        std::shared_ptr<eprosima::fastdds::rtps::TCPv6TransportDescriptor> descriptor =
                std::make_shared<eprosima::fastdds::rtps::TCPv6TransportDescriptor>();

        // Enable TLS
        if (tls_config->is_active())
        {
            enable_tls(descriptor, tls_config, true);
        }

        params.userTransports.push_back(descriptor);

        logDebug(DDSROUTER_DISCOVERYSERVER_PARTICIPANT,
                "Adding TCPv6 Transport to Participant " << this->id() << ".");
    }

    // If has UDP, create descriptor because it has not been created yet
    if (has_udp_ipv4)
    {
        std::shared_ptr<eprosima::fastdds::rtps::UDPv4TransportDescriptor> descriptor =
                std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
        params.userTransports.push_back(descriptor);

        logDebug(DDSROUTER_DISCOVERYSERVER_PARTICIPANT,
                "Adding UDPv4 Transport to Participant " << this->id() << ".");
    }
    if (has_udp_ipv6)
    {
        std::shared_ptr<eprosima::fastdds::rtps::UDPv6TransportDescriptor> descriptor_v6 =
                std::make_shared<eprosima::fastdds::rtps::UDPv6TransportDescriptor>();
        params.userTransports.push_back(descriptor_v6);

        logDebug(DDSROUTER_DISCOVERYSERVER_PARTICIPANT,
                "Adding UDPv6 Transport to Participant " << this->id() << ".");
    }

    logDebug(DDSROUTER_DISCOVERYSERVER_PARTICIPANT,
            "Configured Participant " << this->id() << " with server guid: " <<
            discovery_server_guid_prefix);

    return params;
}

template <class ConfigurationType>
void DiscoveryServerParticipant<ConfigurationType>::enable_tls(
        std::shared_ptr<eprosima::fastdds::rtps::TCPTransportDescriptor> descriptor,
        std::shared_ptr<types::security::TlsConfiguration> tls_configuration,
        bool client /* = false */)
{
    // Apply security ON
    descriptor->apply_security = true;

    // Options
    descriptor->tls_config.add_option(
        eprosima::fastdds::rtps::TCPTransportDescriptor::TLSConfig::TLSOptions::DEFAULT_WORKAROUNDS);
    descriptor->tls_config.add_option(
        eprosima::fastdds::rtps::TCPTransportDescriptor::TLSConfig::TLSOptions::SINGLE_DH_USE);
    descriptor->tls_config.add_option(
        eprosima::fastdds::rtps::TCPTransportDescriptor::TLSConfig::TLSOptions::NO_SSLV2); // not safe

    // Perform verification of the server
    descriptor->tls_config.add_verify_mode(
        eprosima::fastdds::rtps::TCPTransportDescriptor::TLSConfig::TLSVerifyMode::VERIFY_PEER);

    if (client)
    {
        if (!tls_configuration->can_be_client())
        {
            logError(DDSROUTER_DISCOVERYSERVER_PARTICIPANT,
                    "TLS Configuration expected a Client configuration.");
            throw utils::ConfigurationException("TLS Configuration expected a Client configuration.");
        }
        else
        {
            enable_tls_client(descriptor, tls_configuration, true);
        }
    }
    else
    {
        if (!tls_configuration->can_be_server())
        {
            logError(DDSROUTER_DISCOVERYSERVER_PARTICIPANT,
                    "TLS Configuration expected a Server configuration.");
            throw utils::ConfigurationException("TLS Configuration expected a Server configuration.");
        }
        else
        {
            // Add configuration for server
            enable_tls_server(descriptor, tls_configuration);

            // In case it could also be client, add tls config
            if (tls_configuration->can_be_client())
            {
                enable_tls_client(descriptor, tls_configuration, false);
            }
        }
    }

    logDebug(DDSROUTER_DISCOVERYSERVER_PARTICIPANT,
            "TLS configured.");
}

template <class ConfigurationType>
void DiscoveryServerParticipant<ConfigurationType>::enable_tls_client(
        std::shared_ptr<eprosima::fastdds::rtps::TCPTransportDescriptor> descriptor,
        std::shared_ptr<types::security::TlsConfiguration> tls_configuration,
        bool only_client)
{
    std::shared_ptr<types::security::TlsConfigurationClient> tls_configuration_ =
            std::dynamic_pointer_cast<types::security::TlsConfigurationClient>(tls_configuration);

    if (!tls_configuration_)
    {
        utils::tsnh(
            utils::Formatter() << "Error, client available TlsConfiguration does not cast to TlsConfigurationClient.");
    }

    if (only_client)
    {
        // Fail verification if the server has no certificate
        descriptor->tls_config.add_verify_mode(
            eprosima::fastdds::rtps::TCPTransportDescriptor::TLSConfig::TLSVerifyMode::VERIFY_FAIL_IF_NO_PEER_CERT);
    }

    // CA certificate
    descriptor->tls_config.verify_file = tls_configuration_->certificate_authority_file();
}

template <class ConfigurationType>
void DiscoveryServerParticipant<ConfigurationType>::enable_tls_server(
        std::shared_ptr<eprosima::fastdds::rtps::TCPTransportDescriptor> descriptor,
        std::shared_ptr<types::security::TlsConfiguration> tls_configuration)
{
    std::shared_ptr<types::security::TlsConfigurationServer> tls_configuration_ =
            std::dynamic_pointer_cast<types::security::TlsConfigurationServer>(tls_configuration);

    if (!tls_configuration_)
    {
        utils::tsnh(
            utils::Formatter() << "Error, server available TlsConfiguration does not cast to TlsConfigurationServer.");
    }

    // Password
    descriptor->tls_config.password = tls_configuration_->private_key_file_password();
    // Private key
    descriptor->tls_config.private_key_file = tls_configuration_->private_key_file();
    // DDS-Router certificate
    descriptor->tls_config.cert_chain_file = tls_configuration_->certificate_chain_file();
    // DH
    descriptor->tls_config.tmp_dh_file = tls_configuration_->dh_params_file();
}

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_RTPS_DISCOVERYSERVERRTPSROUTERPARTICIPANT_IMPL_IPP_ */
