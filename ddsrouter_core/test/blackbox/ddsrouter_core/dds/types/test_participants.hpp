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
 * @file test_participants.hpp
 */

#ifndef _TEST_BLACKBOX_DDSROUTERCORE_DDS_TYPES_TEST_PARTICIPANTS_HPP_
#define _TEST_BLACKBOX_DDSROUTERCORE_DDS_TYPES_TEST_PARTICIPANTS_HPP_

#include <atomic>
#include <iostream>

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddsrouter_core/configuration/participant/SimpleParticipantConfiguration.hpp>
#include <ddsrouter_core/core/DDSRouter.hpp>
#include <ddsrouter_utils/Log.hpp>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/attributes/SubscriberAttributes.h>

#include "HelloWorld/HelloWorldPubSubTypes.h"
#include "HelloWorldKeyed/HelloWorldKeyedPubSubTypes.h"

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace test {

/**
 * Class used to group into a single working unit a Publisher with a DataWriter and a TypeSupport member corresponding
 * to the HelloWorld datatype
 */
template <class MsgStruct>
class TestPublisher
{
public:

    TestPublisher(
            bool keyed = false)
        : participant_(nullptr)
        , publisher_(nullptr)
        , topic_(nullptr)
        , writer_(nullptr)
        , keyed_(keyed)
    {
    }

    ~TestPublisher()
    {
        if (participant_ != nullptr)
        {
            if (publisher_ != nullptr)
            {
                if (writer_ != nullptr)
                {
                    publisher_->delete_datawriter(writer_);
                }
                participant_->delete_publisher(publisher_);
            }
            if (topic_ != nullptr)
            {
                participant_->delete_topic(topic_);
            }
            eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->delete_participant(participant_);
        }
    }

    //! Initialize the publisher
    bool init(
            uint32_t domain)
    {
        // CREATE THE PARTICIPANT
        eprosima::fastdds::dds::DomainParticipantQos pqos;

        pqos.name("Participant_pub");
        participant_ =
                eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->create_participant(domain, pqos);

        if (participant_ == nullptr)
        {
            return false;
        }

        // REGISTER THE TYPE
        eprosima::fastdds::dds::TypeSupport type;
        if (keyed_)
        {
            type = eprosima::fastdds::dds::TypeSupport(new HelloWorldKeyedPubSubType());
        }
        else
        {
            type = eprosima::fastdds::dds::TypeSupport(new HelloWorldPubSubType());
        }
        type.register_type(participant_);

        // CREATE THE PUBLISHER
        publisher_ = participant_->create_publisher(eprosima::fastdds::dds::PUBLISHER_QOS_DEFAULT, nullptr);

        if (publisher_ == nullptr)
        {
            return false;
        }

        // CREATE THE TOPIC
        std::string type_name = keyed_ ? "HelloWorldKeyed" : "HelloWorld";
        topic_ = participant_->create_topic("DDS-Router-Test", type_name, eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);

        if (topic_ == nullptr)
        {
            return false;
        }

        // CREATE THE WRITER
        // Set memory management policy so it uses realloc
        eprosima::fastdds::dds::DataWriterQos wqos =  eprosima::fastdds::dds::DATAWRITER_QOS_DEFAULT;
        wqos.endpoint().history_memory_policy =
                eprosima::fastrtps::rtps::MemoryManagementPolicy_t::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
        writer_ = publisher_->create_datawriter(topic_, wqos);

        if (writer_ == nullptr)
        {
            return false;
        }

        return true;
    }

    //! Publish a sample
    void publish(
            MsgStruct msg)
    {
        hello_.index(msg.index());
        hello_.message(msg.message());
        ASSERT_TRUE(writer_->write(&hello_));
    }

private:

    MsgStruct hello_;

    eprosima::fastdds::dds::DomainParticipant* participant_;

    eprosima::fastdds::dds::Publisher* publisher_;

    eprosima::fastdds::dds::Topic* topic_;

    eprosima::fastdds::dds::DataWriter* writer_;

    bool keyed_;
};

/**
 * Class used to group into a single working unit a Subscriber with a DataReader, its listener, and a TypeSupport member
 * corresponding to the HelloWorld datatype
 */
template <class MsgStruct>
class TestSubscriber
{
public:

    TestSubscriber(
            bool keyed = false)
        : participant_(nullptr)
        , subscriber_(nullptr)
        , topic_(nullptr)
        , reader_(nullptr)
        , keyed_(keyed)
    {
    }

    ~TestSubscriber()
    {
        if (participant_ != nullptr)
        {
            if (topic_ != nullptr)
            {
                participant_->delete_topic(topic_);
            }
            if (subscriber_ != nullptr)
            {
                if (reader_ != nullptr)
                {
                    subscriber_->delete_datareader(reader_);
                }
                participant_->delete_subscriber(subscriber_);
            }
            eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->delete_participant(participant_);
        }
    }

    //! Initialize the subscriber
    bool init(
            uint32_t domain,
            MsgStruct* msg_should_receive,
            std::atomic<uint32_t>* samples_received)
    {
        // INITIALIZE THE LISTENER
        listener_.init(msg_should_receive, samples_received);

        // CREATE THE PARTICIPANT
        eprosima::fastdds::dds::DomainParticipantQos pqos;
        pqos.name("Participant_sub");
        participant_ =
                eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->create_participant(domain, pqos);

        if (participant_ == nullptr)
        {
            return false;
        }

        // REGISTER THE TYPE
        eprosima::fastdds::dds::TypeSupport type;
        if (keyed_)
        {
            type = eprosima::fastdds::dds::TypeSupport(new HelloWorldKeyedPubSubType());
        }
        else
        {
            type = eprosima::fastdds::dds::TypeSupport(new HelloWorldPubSubType());
        }
        type.register_type(participant_);

        // CREATE THE SUBSCRIBER
        subscriber_ = participant_->create_subscriber(eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT, nullptr);

        if (subscriber_ == nullptr)
        {
            return false;
        }

        // CREATE THE TOPIC
        std::string type_name = keyed_ ? "HelloWorldKeyed" : "HelloWorld";
        topic_ = participant_->create_topic("DDS-Router-Test", type_name, eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);

        if (topic_ == nullptr)
        {
            return false;
        }

        // CREATE THE READER
        // Set memory management policy so it uses realloc
        eprosima::fastdds::dds::DataReaderQos rqos =  eprosima::fastdds::dds::DATAREADER_QOS_DEFAULT;
        rqos.endpoint().history_memory_policy =
                eprosima::fastrtps::rtps::MemoryManagementPolicy_t::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
        reader_ = subscriber_->create_datareader(topic_, rqos, &listener_);

        if (reader_ == nullptr)
        {
            return false;
        }

        return true;
    }

private:

    eprosima::fastdds::dds::DomainParticipant* participant_;

    eprosima::fastdds::dds::Subscriber* subscriber_;

    eprosima::fastdds::dds::Topic* topic_;

    eprosima::fastdds::dds::DataReader* reader_;

    bool keyed_;

    /**
     * Class handling dataflow events
     */
    class SubListener : public eprosima::fastdds::dds::DataReaderListener
    {
    public:

        //! Initialize the listener
        void init(
                MsgStruct* msg_should_receive,
                std::atomic<uint32_t>* samples_received)
        {
            msg_should_receive_ = msg_should_receive;
            samples_received_ = samples_received;
        }

        //! Callback executed when a new sample is received
        void on_data_available(
                eprosima::fastdds::dds::DataReader* reader) override
        {
            bool success = false;
            eprosima::fastdds::dds::SampleInfo info;
            while (reader->take_next_sample(&msg_received_, &info) == ReturnCode_t::RETCODE_OK)
            {
                if (info.instance_state == eprosima::fastdds::dds::ALIVE_INSTANCE_STATE)
                {
                    if (msg_received_.message() == msg_should_receive_->message())
                    {
                        success = true;
                        (*samples_received_)++;
                    }
                }
            }
            ASSERT_TRUE(success);
        }

    private:

        //! Placeholder where received data is stored
        MsgStruct msg_received_;

        //! Reference to the sample sent by the publisher
        MsgStruct* msg_should_receive_;

        //! Reference to received messages counter
        std::atomic<uint32_t>* samples_received_;
    }
    listener_;
};

} /* namespace test */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _TEST_BLACKBOX_DDSROUTERCORE_DDS_TYPES_TEST_PARTICIPANTS_HPP_ */
