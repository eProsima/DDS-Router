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
 * @file DataBrokerLocalParticipant.cpp
 *
 */

#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>

#include <databroker/Address.hpp>
#include <databroker/DataBrokerLocalParticipant.hpp>

namespace eprosima {
namespace databroker {

DataBrokerLocalParticipant::DataBrokerLocalParticipant(
        eprosima::fastdds::dds::DomainParticipantListener* listener,
        DataBrokerLocalParticipantConfiguration configuration)
    : DataBrokerParticipant(listener)
    , configuration_(configuration)
{
}

eprosima::fastrtps::rtps::GuidPrefix_t DataBrokerLocalParticipant::guid()
{
    if (configuration_.discovery_protocol == fastrtps::rtps::DiscoveryProtocol::SERVER)
    {
        return configuration_.server_guid;
    }
    else
    {
        return DataBrokerParticipant::guid();
    }
}

eprosima::fastdds::dds::DomainParticipantQos DataBrokerLocalParticipant::participant_qos()
{
    eprosima::fastdds::dds::DomainParticipantQos pqos = DataBrokerParticipant::participant_qos();

    if (configuration_.discovery_protocol ==
            fastrtps::rtps::DiscoveryProtocol::SERVER)
    {
        // Set this participant as a SERVER
        pqos.wire_protocol().builtin.discovery_config.discoveryProtocol =
                fastrtps::rtps::DiscoveryProtocol::SERVER;

        // Configuring Server Guid
        pqos.wire_protocol().prefix = configuration_.server_guid;

        for (auto address : configuration_.listening_addresses)
        {
            eprosima::fastdds::rtps::Locator server_locator;
            server_locator.kind = LOCATOR_KIND_UDPv4;
            eprosima::fastrtps::rtps::IPLocator::setIPv4(server_locator, address.ip);
            eprosima::fastrtps::rtps::IPLocator::setPhysicalPort(server_locator, address.port);
            pqos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(server_locator);
        }
    }

    return pqos;
}

eprosima::fastdds::dds::DataWriterQos DataBrokerLocalParticipant::datawriter_qos()
{
    eprosima::fastdds::dds::DataWriterQos datawriter_qos = DataBrokerParticipant::datawriter_qos();

    datawriter_qos.publish_mode().kind = eprosima::fastdds::dds::PublishModeQosPolicyKind::ASYNCHRONOUS_PUBLISH_MODE;
    datawriter_qos.reliability().kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    datawriter_qos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;

    return datawriter_qos;
}

eprosima::fastdds::dds::DataReaderQos DataBrokerLocalParticipant::datareader_qos()
{
    eprosima::fastdds::dds::DataReaderQos datareader_qos = DataBrokerParticipant::datareader_qos();

    datareader_qos.reliability().kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::BEST_EFFORT_RELIABILITY_QOS;
    datareader_qos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::VOLATILE_DURABILITY_QOS;

    return datareader_qos;
}

std::string DataBrokerLocalParticipant::type_name_()
{
    if (configuration_.ros)
    {
        return "std_msgs::msg::dds_::String_";
    }
    else
    {
        return DataBrokerParticipant::type_name_();
    }
}

std::string DataBrokerLocalParticipant::topic_mangled_(
        const std::string& topic_name)
{
    if (configuration_.ros)
    {
        return "rt/" + topic_name;
    }
    else
    {
        return DataBrokerParticipant::topic_mangled_(topic_name);
    }
}

std::string DataBrokerLocalParticipant::name()
{
    return "Internal_" + std::string(configuration_.ros ? "ROS_" : "") + "DataBroker_Participant";
}

const DataBrokerParticipantConfiguration& DataBrokerLocalParticipant::get_configuration_()
{
    return configuration_;
}

} /* namespace databroker */
} /* namespace eprosima */
