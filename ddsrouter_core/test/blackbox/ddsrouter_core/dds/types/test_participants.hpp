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

#pragma once

#include <atomic>
#include <iostream>
#include <condition_variable>

#include <fastdds/dds/common/InstanceHandle.hpp>
#include <fastdds/dds/core/detail/DDSReturnCode.hpp>
#include <fastdds/dds/core/detail/DDSReturnCode.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/rtps/attributes/RTPSParticipantAttributes.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.hpp>

#include "HelloWorld/HelloWorldPubSubTypes.hpp"
#include "HelloWorldKeyed/HelloWorldKeyedPubSubTypes.hpp"

#include <gtest/gtest.h>


namespace test {

constexpr const char* TOPIC_NAME = "DDS-Router-Test";

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
            participant_->delete_contained_entities();
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
        topic_ = participant_->create_topic(TOPIC_NAME, type_name, eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);

        if (topic_ == nullptr)
        {
            return false;
        }

        // CREATE THE WRITER
        // Set memory management policy so it uses realloc
        eprosima::fastdds::dds::DataWriterQos wqos =  eprosima::fastdds::dds::DATAWRITER_QOS_DEFAULT;
        wqos.endpoint().history_memory_policy =
                eprosima::fastdds::rtps::MemoryManagementPolicy_t::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
        wqos.history().kind = eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_ALL_HISTORY_QOS;
        writer_ = publisher_->create_datawriter(topic_, wqos, &listener_);

        if (writer_ == nullptr)
        {
            return false;
        }

        return true;
    }

    //! Publish a sample
    eprosima::fastdds::dds::ReturnCode_t publish(
            MsgStruct msg)
    {
        hello_.index(msg.index());
        hello_.message(msg.message());
        return writer_->write(&hello_);
    }

    //! Dispose instance
    eprosima::fastdds::dds::ReturnCode_t dispose_key(
            MsgStruct msg);

    void wait_discovery(
            uint32_t n_subscribers = 1)
    {
        listener_.wait_discovery(n_subscribers);
    }

private:

    MsgStruct hello_;

    eprosima::fastdds::dds::DomainParticipant* participant_;

    eprosima::fastdds::dds::Publisher* publisher_;

    eprosima::fastdds::dds::Topic* topic_;

    eprosima::fastdds::dds::DataWriter* writer_;

    bool keyed_;

    class PubListener : public eprosima::fastdds::dds::DataWriterListener
    {
    public:

        PubListener()
            : discovered(0)
        {
        }

        void wait_discovery(
                uint32_t n_subscribers = 1)
        {
            if (discovered < n_subscribers)
            {
                std::unique_lock<std::mutex> lock(wait_discovery_cv_mtx);
                wait_discovery_cv.wait(lock, [this, n_subscribers]
                        {
                            return discovered >= n_subscribers;
                        });
            }
        }

        void on_publication_matched(
                eprosima::fastdds::dds::DataWriter*,
                const eprosima::fastdds::dds::PublicationMatchedStatus& info)
        {
            if (info.current_count_change == 1)
            {
                discovered = info.current_count;
                wait_discovery_cv.notify_all();
            }
            else if (info.current_count_change == -1)
            {
                discovered = info.current_count;
            }
        }

    private:

        //! Number of DataReaders discovered
        std::atomic<std::uint32_t> discovered;

        //! Protects wait_discovery condition variable
        std::mutex wait_discovery_cv_mtx;

        //! Waits to discovery enough DataReaders
        std::condition_variable wait_discovery_cv;
    }
    listener_;
};

template <>
eprosima::fastdds::dds::ReturnCode_t TestPublisher<HelloWorldKeyed>::publish(
        HelloWorldKeyed msg)
{
    hello_.index(msg.index());
    hello_.message(msg.message());
    hello_.id(msg.id());
    return writer_->write(&hello_);
}

template <>
eprosima::fastdds::dds::ReturnCode_t TestPublisher<HelloWorldKeyed>::dispose_key(
        HelloWorldKeyed msg)
{
    hello_.id(msg.id());
    return writer_->dispose(&hello_, eprosima::fastdds::dds::HANDLE_NIL);
}

/**
 * Class used to group into a single working unit a Subscriber with a DataReader, its listener, and a TypeSupport member
 * corresponding to the HelloWorld datatype
 */
template <class MsgStruct>
class TestSubscriber
{
public:

    TestSubscriber(
            bool keyed = false,
            bool reliable = false)
        : participant_(nullptr)
        , subscriber_(nullptr)
        , topic_(nullptr)
        , reader_(nullptr)
        , keyed_(keyed)
        , reliable_ (reliable)
    {
    }

    ~TestSubscriber()
    {
        if (participant_ != nullptr)
        {
            participant_->delete_contained_entities();
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
        topic_ = participant_->create_topic(TOPIC_NAME, type_name, eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);

        if (topic_ == nullptr)
        {
            return false;
        }

        // CREATE THE READER
        // Set memory management policy so it uses realloc
        eprosima::fastdds::dds::DataReaderQos rqos =  eprosima::fastdds::dds::DATAREADER_QOS_DEFAULT;
        rqos.endpoint().history_memory_policy =
                eprosima::fastdds::rtps::MemoryManagementPolicy_t::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
        rqos.history().kind = eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_ALL_HISTORY_QOS;
        if (reliable_)
        {
            rqos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
            rqos.reliability().kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
        }

        reader_ = subscriber_->create_datareader(topic_, rqos, &listener_);

        if (reader_ == nullptr)
        {
            return false;
        }

        return true;
    }

    void wait_discovery(
            uint32_t n_publishers = 1)
    {
        listener_.wait_discovery(n_publishers);
    }

    uint32_t n_disposed() const
    {
        return listener_.n_key_disposed;
    }

private:

    eprosima::fastdds::dds::DomainParticipant* participant_;

    eprosima::fastdds::dds::Subscriber* subscriber_;

    eprosima::fastdds::dds::Topic* topic_;

    eprosima::fastdds::dds::DataReader* reader_;

    bool keyed_;

    bool reliable_;

    /**
     * Class handling dataflow events
     */
    class SubListener : public eprosima::fastdds::dds::DataReaderListener
    {
    public:

        SubListener()
            : discovered(0)
        {
        }

        //! Initialize the listener
        void init(
                MsgStruct* msg_should_receive_arg,
                std::atomic<uint32_t>* samples_received_arg)
        {
            msg_should_receive = msg_should_receive_arg;
            samples_received = samples_received_arg;
            n_key_disposed = 0;
        }

        void wait_discovery(
                uint32_t n_publishers = 1)
        {
            if (discovered < n_publishers)
            {
                std::unique_lock<std::mutex> lock(wait_discovery_cv_mtx);
                wait_discovery_cv.wait(lock, [this, n_publishers]
                        {
                            return discovered >= n_publishers;
                        });
            }
        }

        //! Callback executed when a new sample is received
        void on_data_available(
                eprosima::fastdds::dds::DataReader* reader) override
        {
            eprosima::fastdds::dds::SampleInfo info;
            while (reader->take_next_sample(&msg_received, &info) == eprosima::fastdds::dds::RETCODE_OK)
            {
                if (info.instance_state == eprosima::fastdds::dds::ALIVE_INSTANCE_STATE)
                {
                    if (msg_received.message() == msg_should_receive->message())
                    {
                        (*samples_received)++;
                    }
                }
                else if (info.instance_state == eprosima::fastdds::dds::NOT_ALIVE_DISPOSED_INSTANCE_STATE)
                {
                    n_key_disposed++;
                }
            }
        }

        void on_subscription_matched(
                eprosima::fastdds::dds::DataReader*,
                const eprosima::fastdds::dds::SubscriptionMatchedStatus& info)
        {
            if (info.current_count_change == 1)
            {
                discovered = info.current_count;
                wait_discovery_cv.notify_all();
            }
            else if (info.current_count_change == -1)
            {
                discovered = info.current_count;
            }
        }

        //! Placeholder where received data is stored
        MsgStruct msg_received;

        std::atomic<std::uint32_t> n_key_disposed;

        //! Reference to the sample sent by the publisher
        MsgStruct* msg_should_receive;

        //! Reference to received messages counter
        std::atomic<uint32_t>* samples_received;

        //! Number of DataWriters discovered
        std::atomic<std::uint32_t> discovered;

        //! Protects wait_discovery condition variable
        std::mutex wait_discovery_cv_mtx;

        //! Waits to discovery enough DataWriters
        std::condition_variable wait_discovery_cv;
    }
    listener_;
};

} /* namespace test */
