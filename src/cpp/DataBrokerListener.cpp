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
 * @file DataBrokerListener.cpp
 *
 */

#include <fastdds/dds/subscriber/SampleInfo.hpp>

#include <databroker/DataBrokerListener.hpp>

namespace eprosima {
namespace databroker {

DataBrokerListener::DataBrokerListener()
    : enabled_(false)
    , local_guid_set_(false)
    , wan_guid_set_(false)
{
}

DataBrokerListener::~DataBrokerListener()
{
}

bool DataBrokerListener::init(
        DataBrokerParticipant* local,
        DataBrokerParticipant* wan)
{
    local_ = local;
    wan_ = wan;

    // Get their guids so in callbacks there is no need to access Participants
    retrieve_wan_guid_prefix_();
    retrieve_local_guid_prefix_();

    // Should always be true
    std::cout << "Listener initialized" << std::endl;
    return local_ && wan_;
}

void DataBrokerListener::block_topic(const std::string& topic)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    topics_blocked_.insert(topic);
}

bool DataBrokerListener::is_topic_blocked(const std::string& topic)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return topics_blocked_.find(topic) != topics_blocked_.end();
}

void DataBrokerListener::on_data_available(
        eprosima::fastdds::dds::DataReader* reader)
{
    StdString data;
    eprosima::fastdds::dds::SampleInfo info;
    if (reader->take_next_sample(&data, &info) == ReturnCode_t::RETCODE_OK)
    {
        if (info.instance_state == eprosima::fastdds::dds::ALIVE_INSTANCE_STATE)
        {
            // If it comes from this DataBroker, avoid resending it (is still read to remove it)
            if (info.sample_identity.writer_guid().guidPrefix != local_guid_prefix_() &&
                info.sample_identity.writer_guid().guidPrefix != wan_guid_prefix_())
            {
                // If the topic of this reader is blocked, read the data but not send it
                if (!is_topic_blocked(reader->get_topicdescription()->get_name()))
                {
                    if (reader->guid().guidPrefix == local_guid_prefix_())
                    {
                        std::cout << "Local Reader in topic " << reader->get_topicdescription()->get_name()
                            << " received data " << data.x() << " from " << info.sample_identity.writer_guid()
                            << std::endl
                            << "Sending through Remote Writer." << std::endl;
                        wan_->send_data(demangle_topic(reader->get_topicdescription()->get_name()), data);
                    }
                    else if (reader->guid().guidPrefix == wan_guid_prefix_())
                    {
                        std::cout << "Remote Reader in topic " << reader->get_topicdescription()->get_name()
                            << " received data " << data.x() << " from " << info.sample_identity.writer_guid()
                            << std::endl
                            << "Sending through Local Writer." << std::endl;
                        local_->send_data(demangle_topic(reader->get_topicdescription()->get_name()), data);
                    }
                    else
                    {
                        std::cerr << "local: " << local_guid_prefix_() << std::endl;
                        std::cerr << "wan: " << wan_guid_prefix_() << std::endl;
                        std::cerr << "this guid: " << reader->guid().guidPrefix << std::endl;
                        std::cerr << "ERROR - Listener attached to a non related Participant" << std::endl;
                    }
                }
            }
        }
        else
        {
            std::cerr << "WARNING - Listener received a non ALIVE instace" << std::endl;
        }
    }
}

void DataBrokerListener::on_subscription_matched(
        eprosima::fastdds::dds::DataReader* reader,
        const eprosima::fastdds::dds::SubscriptionMatchedStatus& info)
{
    if (info.current_count_change > 0)
    {
        if (reader->guid().guidPrefix == local_guid_prefix_())
        {
            std::cout << "Local Reader in topic " << reader->get_topicdescription()->get_name()
                << " matched with Writer " << info.last_publication_handle << std::endl;
        }
        else if (reader->guid().guidPrefix == wan_guid_prefix_())
        {
            std::cout << "External Reader in topic " << reader->get_topicdescription()->get_name()
                << " matched with Writer " << info.last_publication_handle << std::endl;
        }
        else
        {
            std::cerr << "local: " << local_guid_prefix_() << std::endl;
            std::cerr << "wan: " << wan_guid_prefix_() << std::endl;
            std::cerr << "this guid: " << reader->guid().guidPrefix << std::endl;
            std::cerr << "ERROR - Listener attached to a non related Participant" << std::endl;
        }
    }
    else
    {
        if (reader->guid().guidPrefix == local_guid_prefix_())
        {
            std::cout << "Local Reader in topic " << reader->get_topicdescription()->get_name()
                << " unmatched with Writer " << info.last_publication_handle << std::endl;
        }
        else if (reader->guid().guidPrefix == wan_guid_prefix_())
        {
            std::cout << "External Reader in topic " << reader->get_topicdescription()->get_name()
                << " unmatched with Writer " << info.last_publication_handle << std::endl;
        }
        else
        {
            std::cerr << "local: " << local_guid_prefix_() << std::endl;
            std::cerr << "wan: " << wan_guid_prefix_() << std::endl;
            std::cerr << "this guid: " << reader->guid().guidPrefix << std::endl;
            std::cerr << "ERROR - Listener attached to a non related Participant" << std::endl;
        }
    }
}

void DataBrokerListener::on_publication_matched(
        eprosima::fastdds::dds::DataWriter* writer,
        const eprosima::fastdds::dds::PublicationMatchedStatus& info)
{
    if (info.current_count_change > 0)
    {
        if (writer->guid().guidPrefix == local_guid_prefix_())
        {
            std::cout << "Local Writer in topic " << writer->get_topic()->get_name()
                << " matched with Reader " << info.last_subscription_handle << std::endl;
        }
        else if (writer->guid().guidPrefix == wan_guid_prefix_())
        {
            std::cout << "External Writer in topic " << writer->get_topic()->get_name()
                << " matched with Reader " << info.last_subscription_handle << std::endl;
        }
        else
        {
            std::cerr << "local: " << local_guid_prefix_() << std::endl;
            std::cerr << "wan: " << wan_guid_prefix_() << std::endl;
            std::cerr << "this guid: " << writer->guid().guidPrefix << std::endl;
            std::cerr << "ERROR - Listener attached to a non related Participant" << std::endl;
        }
    }
    else
    {
        if (writer->guid().guidPrefix == local_guid_prefix_())
        {
            std::cout << "Local Writer in topic " << writer->get_topic()->get_name()
                << " unmatched with Reader " << info.last_subscription_handle << std::endl;
        }
        else if (writer->guid().guidPrefix == wan_guid_prefix_())
        {
            std::cout << "External Writer in topic " << writer->get_topic()->get_name()
                << " unmatched with Reader " << info.last_subscription_handle << std::endl;
        }
        else
        {
            std::cerr << "local: " << local_guid_prefix_() << std::endl;
            std::cerr << "wan: " << wan_guid_prefix_() << std::endl;
            std::cerr << "this guid: " << writer->guid().guidPrefix << std::endl;
            std::cerr << "ERROR - Listener attached to a non related Participant" << std::endl;
        }
    }
}

void DataBrokerListener::on_participant_discovery(
        eprosima::fastdds::dds::DomainParticipant* participant,
        eprosima::fastrtps::rtps::ParticipantDiscoveryInfo&& info)
{
    if (info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::DISCOVERED_PARTICIPANT)
    {
        if (participant->guid().guidPrefix == local_guid_prefix_())
        {
            std::cout << "Participant found in local network with guid: " << info.info.m_guid << std::endl;
        }
        else if (participant->guid().guidPrefix == wan_guid_prefix_())
        {
            std::cout << "Participant found in external network with guid: " << info.info.m_guid << std::endl;
        }
        else
        {
            std::cerr << "local: " << local_guid_prefix_() << std::endl;
            std::cerr << "wan: " << wan_guid_prefix_() << std::endl;
            std::cerr << "this guid: " << participant->guid().guidPrefix << std::endl;
            std::cerr << "ERROR - Listener attached to a non related Participant" << std::endl;
        }
    }
    else if (info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::DROPPED_PARTICIPANT ||
            info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::REMOVED_PARTICIPANT)
    {
        if (participant->guid().guidPrefix == local_guid_prefix_())
        {
            std::cout << "Participant dropped in local network with guid: " << info.info.m_guid << std::endl;
        }
        else if (participant->guid().guidPrefix == wan_guid_prefix_())
        {
            std::cout << "Participant dropped in external network with guid: " << info.info.m_guid << std::endl;
        }
        else
        {
            std::cerr << "local: " << local_guid_prefix_() << std::endl;
            std::cerr << "wan: " << wan_guid_prefix_() << std::endl;
            std::cerr << "this guid: " << participant->guid().guidPrefix << std::endl;
            std::cerr << "ERROR - Listener attached to a non related Participant" << std::endl;
        }
    }
}

void DataBrokerListener::on_subscriber_discovery(
        eprosima::fastdds::dds::DomainParticipant* participant,
        eprosima::fastrtps::rtps::ReaderDiscoveryInfo&& info)
{
    if (info.status == eprosima::fastrtps::rtps::ReaderDiscoveryInfo::DISCOVERED_READER)
    {
        if (participant->guid().guidPrefix == local_guid_prefix_())
        {
            std::cout << "Subscription found in local network in topic " << info.info.topicName()
                    << " with guid " << info.info.guid().guidPrefix << std::endl;
        }
        else if (participant->guid().guidPrefix == wan_guid_prefix_())
        {
            std::cout << "Subscription found in external network in topic " << info.info.topicName()
                    << " with guid " << info.info.guid().guidPrefix << std::endl;
        }
        else
        {
            std::cerr << "local: " << local_guid_prefix_() << std::endl;
            std::cerr << "wan: " << wan_guid_prefix_() << std::endl;
            std::cerr << "this guid: " << participant->guid().guidPrefix << std::endl;
            std::cerr << "ERROR - Listener attached to a non related Participant" << std::endl;
        }
    }
    else if (info.status == eprosima::fastrtps::rtps::ReaderDiscoveryInfo::REMOVED_READER)
    {
        if (participant->guid().guidPrefix == local_guid_prefix_())
        {
            std::cout << "Subscription dropped in local network in topic " << info.info.topicName()
                    << " with guid " << info.info.guid().guidPrefix << std::endl;
        }
        else if (participant->guid().guidPrefix == wan_guid_prefix_())
        {
            std::cout << "Subscription dropped in external network in topic " << info.info.topicName()
                    << " with guid " << info.info.guid().guidPrefix << std::endl;
        }
        else
        {
            std::cerr << "local: " << local_guid_prefix_() << std::endl;
            std::cerr << "wan: " << wan_guid_prefix_() << std::endl;
            std::cerr << "this guid: " << participant->guid().guidPrefix << std::endl;
            std::cerr << "ERROR - Listener attached to a non related Participant" << std::endl;
        }
    }
}

void DataBrokerListener::on_publisher_discovery(
        eprosima::fastdds::dds::DomainParticipant* participant,
        eprosima::fastrtps::rtps::WriterDiscoveryInfo&& info)
{
    if (info.status == eprosima::fastrtps::rtps::WriterDiscoveryInfo::DISCOVERED_WRITER)
    {
        if (participant->guid().guidPrefix == local_guid_prefix_())
        {
            std::cout << "Publication found in local network in topic " << info.info.topicName()
                    << " with guid " << info.info.guid().guidPrefix << std::endl;
        }
        else if (participant->guid().guidPrefix == wan_guid_prefix_())
        {
            std::cout << "Publication found in external network in topic " << info.info.topicName()
                    << " with guid " << info.info.guid().guidPrefix << std::endl;
        }
        else
        {
            std::cerr << "local: " << local_guid_prefix_() << std::endl;
            std::cerr << "wan: " << wan_guid_prefix_() << std::endl;
            std::cerr << "this guid: " << participant->guid().guidPrefix << std::endl;
            std::cerr << "ERROR - Listener attached to a non related Participant" << std::endl;
        }
    }
    else if (info.status == eprosima::fastrtps::rtps::WriterDiscoveryInfo::REMOVED_WRITER)
    {
        if (participant->guid().guidPrefix == local_guid_prefix_())
        {
            std::cout << "Publication dropped in local network in topic " << info.info.topicName()
                    << " with guid " << info.info.guid().guidPrefix << std::endl;
        }
        else if (participant->guid().guidPrefix == wan_guid_prefix_())
        {
            std::cout << "Publication dropped in external network in topic " << info.info.topicName()
                    << " with guid " << info.info.guid().guidPrefix << std::endl;
        }
        else
        {
            std::cerr << "local: " << local_guid_prefix_() << std::endl;
            std::cerr << "wan: " << wan_guid_prefix_() << std::endl;
            std::cerr << "this guid: " << participant->guid().guidPrefix << std::endl;
            std::cerr << "ERROR - Listener attached to a non related Participant" << std::endl;
        }
    }
}

eprosima::fastrtps::rtps::GuidPrefix_t DataBrokerListener::local_guid_prefix_()
{
    retrieve_local_guid_prefix_();
    return local_guid_;
}

eprosima::fastrtps::rtps::GuidPrefix_t DataBrokerListener::wan_guid_prefix_()
{
    retrieve_wan_guid_prefix_();
    return wan_guid_;
}

void DataBrokerListener::retrieve_local_guid_prefix_()
{
    if (!local_guid_set_)
    {
        local_guid_ = local_->guid();
        if (local_guid_ != eprosima::fastrtps::rtps::GUID_t::unknown().guidPrefix)
        {
            local_guid_set_ = true;
        }
    }
}

void DataBrokerListener::retrieve_wan_guid_prefix_()
{
    if (!wan_guid_set_)
    {
        wan_guid_ = wan_->guid();
        if (wan_guid_ != eprosima::fastrtps::rtps::GUID_t::unknown().guidPrefix)
        {
            wan_guid_set_ = true;
        }
    }
}

void DataBrokerListener::on_data_on_readers(
        eprosima::fastdds::dds::Subscriber* sub)
{
    std::cout << "MASK NEEDED" << std::endl;
}

std::string DataBrokerListener::demangle_topic(const std::string& topic_name)
{
    if (topic_name.rfind("rt/", 0) == 0)
    {
        return topic_name.substr(3);
    }
    else
    {
        return topic_name;
    }
}

} /* namespace databroker */
} /* namespace eprosima */
