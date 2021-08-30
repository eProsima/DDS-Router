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
 * @file DataBrokerParticipant.hpp
 *
 */

#ifndef EPROSIMA_DATABROKER_DATABROKERPARTICIPANT_HPP
#define EPROSIMA_DATABROKER_DATABROKERPARTICIPANT_HPP

#include <mutex>

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantListener.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>

#include <StdString/StdString.h>

namespace eprosima {
namespace databroker {

class DataBrokerParticipant
{
public:

    DataBrokerParticipant(
            eprosima::fastdds::dds::DomainParticipantListener* listener,
            uint32_t domain = 0,
            std::string name = "DataBroker Participant");

    virtual ~DataBrokerParticipant();

    virtual bool init(
            eprosima::fastdds::dds::DomainParticipantQos pqos);

    virtual bool enable();

    virtual void add_topic(
            const std::string& topic_name);

    virtual void stop_topic(
            const std::string& topic);

    virtual void send_data(
            const std::string& topic,
            StdString& data);

    virtual eprosima::fastrtps::rtps::GuidPrefix_t guid();

    eprosima::fastdds::dds::DomainParticipantQos default_participant_qos();

protected:

    virtual eprosima::fastdds::dds::DataWriterQos default_datawriter_qos();

    virtual eprosima::fastdds::dds::DataReaderQos default_datareader_qos();

    virtual bool register_type_();

    virtual std::string topic_mangled_(
            const std::string& topic_name);

    virtual std::string type_name_();

    virtual eprosima::fastdds::dds::Topic* get_topic_(
            const std::string& topic_name);

    virtual std::string name();

    eprosima::fastdds::dds::DomainParticipant* participant_;
    eprosima::fastdds::dds::Publisher* publisher_;
    eprosima::fastdds::dds::Subscriber* subscriber_;

    std::map<std::string /*topic*/, eprosima::fastdds::dds::DataWriter*> datawriters_;
    std::map<std::string /*topic*/, eprosima::fastdds::dds::DataReader*> datareaders_;
    std::map<std::string /*topic*/, eprosima::fastdds::dds::Topic*> topics_;

    eprosima::fastdds::dds::DomainParticipantListener* listener_;

    eprosima::fastdds::dds::TypeSupport type_;

    std::recursive_mutex mutex_;

    std::string name_;
    uint32_t domain_;
    bool enabled_;
};

} /* namespace databroker */
} /* namespace eprosima */

#endif /* EPROSIMA_DATABROKER_DATABROKERPARTICIPANT_HPP */
