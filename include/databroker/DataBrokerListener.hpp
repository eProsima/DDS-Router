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
 * @file DataBrokerListener.hpp
 *
 */

#ifndef EPROSIMA_DATABROKER_DATABROKERLISTENER_HPP
#define EPROSIMA_DATABROKER_DATABROKERLISTENER_HPP

#include <fastdds/dds/domain/DomainParticipantListener.hpp>

#include <databroker/DataBrokerParticipant.hpp>

namespace eprosima {
namespace databroker {

class DataBrokerListener : public eprosima::fastdds::dds::DomainParticipantListener
{
public:

    DataBrokerListener();

    virtual ~DataBrokerListener();

    bool init(
            DataBrokerParticipant* local,
            DataBrokerParticipant* wan);

    void block_topic(
            const std::string& topic);

    void DataBrokerListener::unblock_topic(
            const std::string& topic);

    void on_data_available(
            eprosima::fastdds::dds::DataReader* reader) override;


    void on_subscription_matched(
            eprosima::fastdds::dds::DataReader* reader,
            const eprosima::fastdds::dds::SubscriptionMatchedStatus& info) override;

    void on_publication_matched(
            eprosima::fastdds::dds::DataWriter* writer,
            const eprosima::fastdds::dds::PublicationMatchedStatus& info) override;

    void on_participant_discovery(
            eprosima::fastdds::dds::DomainParticipant* participant,
            eprosima::fastrtps::rtps::ParticipantDiscoveryInfo&& info) override;

    void on_subscriber_discovery(
            eprosima::fastdds::dds::DomainParticipant* participant,
            eprosima::fastrtps::rtps::ReaderDiscoveryInfo&& info) override;

    void on_publisher_discovery(
            eprosima::fastdds::dds::DomainParticipant* participant,
            eprosima::fastrtps::rtps::WriterDiscoveryInfo&& info) override;

protected:

    bool is_topic_blocked(
            const std::string& topic);

    eprosima::fastrtps::rtps::GuidPrefix_t local_guid_prefix_();

    eprosima::fastrtps::rtps::GuidPrefix_t wan_guid_prefix_();

    void retrieve_local_guid_prefix_();

    void retrieve_wan_guid_prefix_();

    static std::string demangle_topic(
            const std::string& topic_name);

    DataBrokerParticipant* local_;
    DataBrokerParticipant* wan_;

    std::set<std::string> topics_blocked_;

    eprosima::fastrtps::rtps::GuidPrefix_t local_guid_;
    eprosima::fastrtps::rtps::GuidPrefix_t wan_guid_;
    bool local_guid_set_;
    bool wan_guid_set_;

    bool enabled_;

    std::recursive_mutex mutex_;
};

} /* namespace databroker */
} /* namespace eprosima */

#endif /* EPROSIMA_DATABROKER_DATABROKERLISTENER_HPP */
