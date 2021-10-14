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
 * @file DataBrokerParticipant.cpp
 *
 */

#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/TCPv4TransportDescriptor.h>

#include <databroker/DataBrokerWANParticipant.hpp>
#include <databroker/Address.hpp>

namespace eprosima {
namespace databroker {

DataBrokerWANParticipant::DataBrokerWANParticipant(
        eprosima::fastdds::dds::DomainParticipantListener* listener,
        DataBrokerWANParticipantConfiguration configuration)
    : DataBrokerParticipant(listener)
    , configuration_(configuration)
{
}

eprosima::fastrtps::rtps::GuidPrefix_t DataBrokerWANParticipant::guid()
{
    return configuration_.server_guid;
}

void DataBrokerWANParticipant::enable_tls_(
        std::shared_ptr<eprosima::fastdds::rtps::TCPv4TransportDescriptor> descriptor)
{
    // Apply security ON
    descriptor->apply_security = true;

    // Private key
    descriptor->tls_config.password = configuration_.tls_password;
    descriptor->tls_config.private_key_file = configuration_.tls_private_key;

    // Own certificate and valid certificates
    descriptor->tls_config.cert_chain_file = configuration_.tls_cert;
    descriptor->tls_config.verify_file = configuration_.tls_ca_cert;

    // DH
    descriptor->tls_config.tmp_dh_file = configuration_.tls_dh_params;

    // Options
    descriptor->tls_config.add_option(
        eprosima::fastdds::rtps::TCPTransportDescriptor::TLSConfig::TLSOptions::DEFAULT_WORKAROUNDS);
    descriptor->tls_config.add_option(
        eprosima::fastdds::rtps::TCPTransportDescriptor::TLSConfig::TLSOptions::SINGLE_DH_USE);
    descriptor->tls_config.add_option(
        eprosima::fastdds::rtps::TCPTransportDescriptor::TLSConfig::TLSOptions::NO_SSLV2); // not safe
    descriptor->tls_config.verify_mode =
            eprosima::fastdds::rtps::TCPTransportDescriptor::TLSConfig::TLSVerifyMode::VERIFY_PEER;

    logInfo(DATABROKER, "External Discovery Server configured with TLS");
}

eprosima::fastdds::dds::DomainParticipantQos DataBrokerWANParticipant::participant_qos()
{
    eprosima::fastdds::dds::DomainParticipantQos pqos = DataBrokerParticipant::participant_qos();

    // Configuring Server Guid
    pqos.wire_protocol().prefix = configuration_.server_guid;

    logInfo(DATABROKER, "External Discovery Server set with guid " << configuration_.server_guid);

    if (!configuration_.udp)
    {
        // In case of using TCP, configure listening address
        // Create TCPv4 transport
        if (configuration_.listening_addresses.empty())
        {
            std::shared_ptr<eprosima::fastdds::rtps::TCPv4TransportDescriptor> descriptor =
                    std::make_shared<eprosima::fastdds::rtps::TCPv4TransportDescriptor>();

            if (configuration_.tls)
            {
                enable_tls_(descriptor);
            }

            pqos.transport().user_transports.push_back(descriptor);
        }
        else
        {
            for (auto address : configuration_.listening_addresses)
            {
                std::shared_ptr<eprosima::fastdds::rtps::TCPv4TransportDescriptor> descriptor =
                        std::make_shared<eprosima::fastdds::rtps::TCPv4TransportDescriptor>();

                descriptor->add_listener_port(address.port);
                descriptor->set_WAN_address(address.ip);

                descriptor->sendBufferSize = 0;
                descriptor->receiveBufferSize = 0;

                if (configuration_.tls)
                {
                    enable_tls_(descriptor);
                }

                pqos.transport().user_transports.push_back(descriptor);

                logInfo(DATABROKER, "External Discovery Server configured TCP listening address " << address);
            }

        }
    }

    for (auto address : configuration_.listening_addresses)
    {
        // Create Locator
        eprosima::fastrtps::rtps::Locator_t locator;
        locator.kind = configuration_.udp ? LOCATOR_KIND_UDPv4 : LOCATOR_KIND_TCPv4;

        eprosima::fastrtps::rtps::IPLocator::setIPv4(locator, address.ip);
        eprosima::fastrtps::rtps::IPLocator::setWan(locator, address.ip);
        eprosima::fastrtps::rtps::IPLocator::setLogicalPort(locator, address.port);
        eprosima::fastrtps::rtps::IPLocator::setPhysicalPort(locator, address.port);

        pqos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(locator);

        logInfo(DATABROKER, "External Discovery Server configured listening locator " << locator);
    }

    // Configure connection addresses
    for (auto address : configuration_.connection_addresses)
    {
        eprosima::fastrtps::rtps::RemoteServerAttributes server_attr;

        // Set Server GUID
        server_attr.guidPrefix = address.guid;

        // Discovery server locator configuration TCP
        eprosima::fastrtps::rtps::Locator_t locator;
        locator.kind = configuration_.udp ? LOCATOR_KIND_UDPv4 : LOCATOR_KIND_TCPv4;

        eprosima::fastrtps::rtps::IPLocator::setIPv4(locator, address.ip);
        eprosima::fastrtps::rtps::IPLocator::setLogicalPort(locator, address.port);
        eprosima::fastrtps::rtps::IPLocator::setPhysicalPort(locator, address.port);
        server_attr.metatrafficUnicastLocatorList.push_back(locator);

        pqos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(server_attr);

        logInfo(DATABROKER, "External Discovery Server configure connection locator " << locator
                                                                                      << " to server "
                                                                                      << server_attr.guidPrefix);
    }

    // Set this participant as a SERVER
    pqos.wire_protocol().builtin.discovery_config.discoveryProtocol =
            fastrtps::rtps::DiscoveryProtocol::SERVER;

    pqos.wire_protocol().builtin.discovery_config.leaseDuration = eprosima::fastrtps::c_TimeInfinite;

    return pqos;
}

eprosima::fastdds::dds::DataWriterQos DataBrokerWANParticipant::datawriter_qos()
{
    eprosima::fastdds::dds::DataWriterQos datawriter_qos = DataBrokerParticipant::datawriter_qos();

    datawriter_qos.publish_mode().kind = eprosima::fastdds::dds::PublishModeQosPolicyKind::ASYNCHRONOUS_PUBLISH_MODE;
    datawriter_qos.reliability().kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    datawriter_qos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;

    return datawriter_qos;
}

eprosima::fastdds::dds::DataReaderQos DataBrokerWANParticipant::datareader_qos()
{
    eprosima::fastdds::dds::DataReaderQos datareader_qos = DataBrokerParticipant::datareader_qos();

    datareader_qos.reliability().kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    datareader_qos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;

    return datareader_qos;
}

std::string DataBrokerWANParticipant::name()
{
    return "External_DataBroker_Participant";
}

const DataBrokerParticipantConfiguration& DataBrokerWANParticipant::get_configuration_()
{
    return configuration_;
}

} /* namespace databroker */
} /* namespace eprosima */
