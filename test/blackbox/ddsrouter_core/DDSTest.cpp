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

#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

#include <gtest_aux.hpp>
#include <gtest/gtest.h>
#include <TestLogHandler.hpp>

#include <ddsrouter/core/DDSRouter.hpp>
#include <ddsrouter/types/Log.hpp>

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

#include "HelloWorldPubSubTypes.h"
#include "HelloWorldKeyedPubSubTypes.h"

using namespace eprosima::ddsrouter;

constexpr const uint32_t SAMPLES_TO_SEND = 10;

/**
 * Class used to group into a single working unit a Publisher with a DataWriter and a TypeSupport member corresponding
 * to the HelloWorld datatype
 */
class HelloWorldPublisher
{
public:

    HelloWorldPublisher(
            bool keyed = false)
        : participant_(nullptr)
        , publisher_(nullptr)
        , topic_(nullptr)
        , writer_(nullptr)
        , keyed_(keyed)
    {
    }

    ~HelloWorldPublisher()
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
        if (keyed_)
        {
            topic_ = participant_->create_topic("HelloWorldTopic", "HelloWorldKeyed",
                            eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
        }
        else
        {
            topic_ = participant_->create_topic("HelloWorldTopic", "HelloWorld",
                            eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);

        }

        if (topic_ == nullptr)
        {
            return false;
        }

        // CREATE THE WRITER
        writer_ = publisher_->create_datawriter(topic_, eprosima::fastdds::dds::DATAWRITER_QOS_DEFAULT);

        if (writer_ == nullptr)
        {
            return false;
        }

        return true;
    }

    //! Publish a sample
    void publish(
            HelloWorld msg)
    {
        hello_.index(msg.index());
        hello_.message(msg.message());
        writer_->write(&hello_);
        std::cout << "Message " << hello_.message() << " " << hello_.index() << " SENT" << std::endl;
    }

private:

    HelloWorld hello_;

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
class HelloWorldSubscriber
{
public:

    HelloWorldSubscriber(
            bool keyed = false)
        : participant_(nullptr)
        , subscriber_(nullptr)
        , topic_(nullptr)
        , reader_(nullptr)
        , keyed_(keyed)
        , data_received_(false)
    {
    }

    ~HelloWorldSubscriber()
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
            HelloWorld* msg_should_receive,
            std::condition_variable* reception_cv,
            std::mutex* reception_cv_mtx)
    {
        // INITIALIZE THE LISTENER
        listener_.init(this, msg_should_receive, reception_cv, reception_cv_mtx);

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
        if (keyed_)
        {
            topic_ = participant_->create_topic("HelloWorldTopic", "HelloWorldKeyed",
                            eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
        }
        else
        {
            topic_ = participant_->create_topic("HelloWorldTopic", "HelloWorld",
                            eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);

        }

        if (topic_ == nullptr)
        {
            return false;
        }

        // CREATE THE READER
        reader_ = subscriber_->create_datareader(topic_, eprosima::fastdds::dds::DATAREADER_QOS_DEFAULT, &listener_);

        if (reader_ == nullptr)
        {
            return false;
        }

        return true;
    }

    bool data_received()
    {
        return data_received_;
    }

    void data_received(
            bool new_value)
    {
        data_received_ = new_value;
    }

private:

    eprosima::fastdds::dds::DomainParticipant* participant_;

    eprosima::fastdds::dds::Subscriber* subscriber_;

    eprosima::fastdds::dds::Topic* topic_;

    eprosima::fastdds::dds::DataReader* reader_;

    bool keyed_;

    //! Attribute set to true when new data is received
    std::atomic<bool> data_received_;

    /**
     * Class handling dataflow events
     */
    class SubListener : public eprosima::fastdds::dds::DataReaderListener
    {
    public:

        //! Initialize the listener
        void init(
                HelloWorldSubscriber* subscriber,
                HelloWorld* msg_should_receive,
                std::condition_variable* reception_cv,
                std::mutex* reception_cv_mtx)
        {
            subscriber_ = subscriber;
            msg_should_receive_ = msg_should_receive;
            reception_cv_ = reception_cv;
            reception_cv_mtx_ = reception_cv_mtx;
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
                    std::cout << "Message " << msg_received_.message() << " " << msg_received_.index() << " RECEIVED" <<
                        std::endl;
                    if (msg_received_.index() == msg_should_receive_->index() &&
                            msg_received_.message() == msg_should_receive_->message())
                    {
                        success = true;
                        {
                            std::lock_guard<std::mutex> lk(*reception_cv_mtx_);
                            subscriber_->data_received(true);
                        }
                        reception_cv_->notify_one();
                    }
                }
            }
            ASSERT_TRUE(success);
        }

    private:

        //! Reference to the subscriber object owning this listener
        HelloWorldSubscriber* subscriber_;

        //! Placeholder where received data is stored
        HelloWorld msg_received_;

        //! Reference to the sample sent by the publisher
        HelloWorld* msg_should_receive_;

        //! Reference to condition variable used to synchronize data flow
        std::condition_variable* reception_cv_;

        //! Reference to mutex managing access to attribute \c data_received_
        std::mutex* reception_cv_mtx_;
    }
    listener_;
};

/**
 * Test whole DDSRouter initialization by initializing two SimpleParticipants
 */
TEST(DDSTest, simple_initialization)
{
    test::TestLogHandler test_log_handler; // Check there are no warning or errors

    // Load configuration
    RawConfiguration router_configuration =
            load_configuration_from_file("resources/dds_test_simple_configuration.yaml");

    // Create DDSRouter entity
    DDSRouter router(router_configuration);

    // Let test finish without failing
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains
 */
TEST(DDSTest, end_to_end_communication)
{
    test::TestLogHandler test_log_handler; // Check there are no warning or errors

    uint32_t samples_sent = 0;

    HelloWorld msg;
    msg.message("HelloWorld");

    //! Condition variable used to synchronize data flow
    std::condition_variable reception_cv;
    //! Mutex managing access to subscriber's \c data_received_ attribute
    std::mutex reception_cv_mtx;

    // Create DDS Publisher in domain 0
    HelloWorldPublisher publisher;
    ASSERT_TRUE(publisher.init(0));

    // Create DDS Subscriber in domain 1
    HelloWorldSubscriber subscriber;
    ASSERT_TRUE(subscriber.init(1, &msg, &reception_cv, &reception_cv_mtx));

    // Load configuration containing two Simple Participants, one in domain 0 and another one in domain 1
    RawConfiguration router_configuration =
            load_configuration_from_file("resources/dds_test_simple_configuration.yaml");

    // Create DDSRouter entity
    DDSRouter router(router_configuration);
    router.start();

    // Wait for the endpoints to match before sending any data
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Start publishing
    while (samples_sent < SAMPLES_TO_SEND)
    {
        subscriber.data_received(false);
        msg.index(++samples_sent);
        publisher.publish(msg);
        std::unique_lock<std::mutex> lck(reception_cv_mtx);
        reception_cv.wait(lck, [&]
                {
                    return subscriber.data_received();
                });
    }

    router.stop();
}

/**
 * Test communication in HelloWorldKeyed topic between two DDS participants created in different domains
 */
TEST(DDSTest, end_to_end_communication_keyed)
{
    test::TestLogHandler test_log_handler; // Check there are no warning or errors

    uint32_t samples_sent = 0;

    HelloWorld msg;
    msg.message("HelloWorldKeyed");

    //! Condition variable used to synchronize data flow
    std::condition_variable reception_cv;
    //! Mutex managing access to subscriber's \c data_received_ attribute
    std::mutex reception_cv_mtx;

    // Create DDS Publisher in domain 0
    HelloWorldPublisher publisher(true);
    ASSERT_TRUE(publisher.init(0));

    // Create DDS Subscriber in domain 1
    HelloWorldSubscriber subscriber(true);
    ASSERT_TRUE(subscriber.init(1, &msg, &reception_cv, &reception_cv_mtx));

    // Load configuration containing two Simple Participants, one in domain 0 and another one in domain 1
    RawConfiguration router_configuration =
            load_configuration_from_file("resources/dds_test_simple_configuration.yaml");

    // Create DDSRouter entity
    DDSRouter router(router_configuration);
    router.start();

    // Wait for the endpoints to match before sending any data
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Start publishing
    while (samples_sent < SAMPLES_TO_SEND)
    {
        subscriber.data_received(false);
        msg.index(++samples_sent);
        publisher.publish(msg);
        std::unique_lock<std::mutex> lck(reception_cv_mtx);
        reception_cv.wait(lck, [&]
                {
                    return subscriber.data_received();
                });
    }

    router.stop();
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
