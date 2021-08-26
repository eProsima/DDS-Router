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
 * @file DataBroker.cpp
 *
 */

#include <stdlib.h>     //for using the function sleep

#include <fastdds/dds/log/Log.hpp>
#include <fastdds/rtps/transport/TCPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>

#include <databroker/DataBroker.hpp>

namespace eprosima {
namespace databroker {

DataBroker::DataBroker(
        const uint32_t domain,
        const eprosima::fastrtps::rtps::GuidPrefix_t& server_guid,
        const std::vector<Address>& listening_address,
        const std::vector<Address>& connection_addresses,
        bool internal_ros,
        bool udp)
    : server_guid_(server_guid)
    , listening_addresses_(listening_address)
    , connection_addresses_(connection_addresses)
    , udp_(udp)
    , enabled_(false)
{
    logInfo(DATABROKER, "Creating DataBroker instance");

    wan_ = new DataBrokerParticipant(&listener_, domain, "External_DataBroker_Participant");

    if (internal_ros)
    {
        local_ = new DataBrokerROSParticipant(&listener_, domain, "Internal_ROS2_DataBroker_Participant");
    }
    else
    {
        local_ = new DataBrokerParticipant(&listener_, domain, "Internal_DataBroker_Participant");
    }
}

DataBroker::~DataBroker()
{
    logInfo(DATABROKER, "Destroying DataBroker instance");

    delete wan_;
    delete local_;
}

bool DataBroker::init(const std::vector<std::string>& initial_topics)
{
    logInfo(DATABROKER, "Intializing DataBroker");

    if (!enabled_)
    {
        listener_.init(local_, wan_);

        if (!wan_->init(wan_participant_qos()))
        {
            std::cerr << "Error Initializing External Participant" << std::endl;
            return false;
        }

        if (!local_->init(default_participant_qos()))
        {
            std::cerr << "Error Initializing Internal Participant" << std::endl;
            return false;
        }

        // When initializing the DataBroker, add the topics that are already set
        for (auto topic : initial_topics)
        {
            add_topic_(topic);
        }
    }

    enabled_ = true;

    logInfo(DATABROKER, "DataBroker initialized");

    return true;
}

bool DataBroker::run(bool interactive, uint32_t seconds /* = 0 */)
{
    if (!enabled_)
    {
        std::cerr << "WARNING DataBroker running without being initialized" << std::endl;
        init(std::vector<std::string>());
    }

    if (interactive)
    {
        return run_interactive();
    }
    else
    {
        return run_time(seconds);
    }
}
bool DataBroker::run_interactive()
{
    logInfo(DATABROKER, "Running DataBroker in interactive mode");
    logWarning(DATABROKER, "Not implemented yet. Press enter to exit.");

    std::string input;
    std::cin >> input;

    // TODO

    return false;
}

bool DataBroker::run_time(const uint32_t seconds)
{
    if (seconds > 0)
    {
        logInfo(DATABROKER, "Running DataBroker for " << seconds << " seconds");
        std::this_thread::sleep_for(std::chrono::seconds(seconds));
    }
    else
    {
        logInfo(DATABROKER, "Running DataBroker until SIGINT is received");
        std::this_thread::sleep_until(std::chrono::time_point<std::chrono::system_clock>::max());
    }

    return true;
}

// TODO decide default qos
eprosima::fastdds::dds::DomainParticipantQos DataBroker::default_participant_qos()
{
    return eprosima::fastdds::dds::DomainParticipantQos();
}

// TODO add debug traces
eprosima::fastdds::dds::DomainParticipantQos DataBroker::wan_participant_qos()
{
    eprosima::fastdds::dds::DomainParticipantQos pqos = default_participant_qos();

    // Configuring Server Guid
    pqos.wire_protocol().prefix = server_guid_;

    logInfo(DATABROKER, "External Discovery Server set with guid " << server_guid_);

    for (auto address : listening_addresses_)
    {
        // Configuring transport
        if (!udp_)
        {
            // In case of using TCP, configure listening address
            // Create TCPv4 transport
            std::shared_ptr<eprosima::fastdds::rtps::TCPv4TransportDescriptor> descriptor =
                std::make_shared<eprosima::fastdds::rtps::TCPv4TransportDescriptor>();

            descriptor->add_listener_port(address.port);
            descriptor->set_WAN_address(address.ip);

            descriptor->sendBufferSize = 0;
            descriptor->receiveBufferSize = 0;

            pqos.transport().user_transports.push_back(descriptor);

            logInfo(DATABROKER, "External Discovery Server configure TCP listening address " << address);
        }

        // Create Locator
        eprosima::fastrtps::rtps::Locator_t locator;
        locator.kind = udp_ ? LOCATOR_KIND_UDPv4 : LOCATOR_KIND_TCPv4;

        eprosima::fastrtps::rtps::IPLocator::setIPv4(locator, address.ip);
        eprosima::fastrtps::rtps::IPLocator::setWan(locator, address.ip);
        eprosima::fastrtps::rtps::IPLocator::setLogicalPort(locator, address.port);
        eprosima::fastrtps::rtps::IPLocator::setPhysicalPort(locator, address.port);

        pqos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(locator);

        logInfo(DATABROKER, "External Discovery Server configure listening locator " << locator);
    }

    // Configure connection addresses
    for (auto address : connection_addresses_)
    {
        eprosima::fastrtps::rtps::RemoteServerAttributes server_attr;

        // Set Server GUID
        server_attr.guidPrefix = address.guid;

        // Discovery server locator configuration TCP
        eprosima::fastrtps::rtps::Locator_t locator;
        locator.kind = udp_ ? LOCATOR_KIND_UDPv4 : LOCATOR_KIND_TCPv4;

        eprosima::fastrtps::rtps::IPLocator::setIPv4(locator, address.ip);
        eprosima::fastrtps::rtps::IPLocator::setLogicalPort(locator, address.port);
        eprosima::fastrtps::rtps::IPLocator::setPhysicalPort(locator, address.port);
        server_attr.metatrafficUnicastLocatorList.push_back(locator);

        pqos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(server_attr);

        logInfo(DATABROKER, "External Discovery Server configure connection locator " << locator);
    }

    // TODO decide the discovery server configuration
    pqos.wire_protocol().builtin.discovery_config.leaseDuration = fastrtps::c_TimeInfinite;
    pqos.wire_protocol().builtin.discovery_config.leaseDuration_announcementperiod =
            fastrtps::Duration_t(2, 0);

    // Set this participant as a SERVER
    pqos.wire_protocol().builtin.discovery_config.discoveryProtocol =
        fastrtps::rtps::DiscoveryProtocol::SERVER;

    return pqos;
}

void DataBroker::add_topic_(const std::string& topic)
{
    std::cout << "Adding topic " << topic << " to whitelist" << std::endl;

    topics_[topic] = true;
    local_->add_topic(topic);
    wan_->add_topic(topic);
}

void DataBroker::remove_topic_(const std::string& topic)
{
    topics_[topic] = false;
    listener_.block_topic(topic);
}

} /* namespace databroker */
} /* namespace eprosima */
