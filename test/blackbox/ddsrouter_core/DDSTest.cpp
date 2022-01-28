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
        std::string type_name = keyed_ ? "HelloWorldKeyed" : "HelloWorld";
        topic_ = participant_->create_topic("HelloWorldTopic", type_name, eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);

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
        std::string type_name = keyed_ ? "HelloWorldKeyed" : "HelloWorld";
        topic_ = participant_->create_topic("HelloWorldTopic", type_name, eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);

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
 * Test communication between two DDS Participants hosted in the same device, but which are at different DDS domains.
 * This is accomplished by using a DDS Router instance with a Simple Participant deployed at each domain.
 */
void test_local_communication(std::string config_path, bool keyed = false)
{
    uint32_t samples_sent = 0;

    HelloWorld msg;
    msg.message("HelloWorld");

    //! Condition variable used to synchronize data flow
    std::condition_variable reception_cv;
    //! Mutex managing access to subscriber's \c data_received_ attribute
    std::mutex reception_cv_mtx;

    // Create DDS Publisher in domain 0
    HelloWorldPublisher publisher(keyed);
    ASSERT_TRUE(publisher.init(0));

    // Create DDS Subscriber in domain 1
    HelloWorldSubscriber subscriber(keyed);
    ASSERT_TRUE(subscriber.init(1, &msg, &reception_cv, &reception_cv_mtx));

    // Load configuration containing two Simple Participants, one in domain 0 and another one in domain 1
    RawConfiguration router_configuration =
            load_configuration_from_file(config_path);

    // Create DDSRouter entity
    DDSRouter router(router_configuration);
    router.start();

    // Wait for the endpoints to match before sending any data
    std::this_thread::sleep_for(std::chrono::seconds(5));

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
 * Test communication between two DDS Participants hosted in the same device, but which are at different DDS domains.
 * This is accomplished by connecting two WAN Participants belonging to different DDS Router instances. These router
 * instances communicate with the DDS Participants through Simple Participants deployed at those domains.
 */
void test_WAN_communication(std::string server_config_path, std::string client_config_path, bool keyed = false)
{
    uint32_t samples_sent = 0;

    HelloWorld msg;
    msg.message("HelloWorld");

    //! Condition variable used to synchronize data flow
    std::condition_variable reception_cv;
    //! Mutex managing access to subscriber's \c data_received_ attribute
    std::mutex reception_cv_mtx;

    // Create DDS Publisher in domain 0
    HelloWorldPublisher publisher(keyed);
    ASSERT_TRUE(publisher.init(0));

    // Create DDS Subscriber in domain 1
    HelloWorldSubscriber subscriber(keyed);
    ASSERT_TRUE(subscriber.init(1, &msg, &reception_cv, &reception_cv_mtx));

    // Load configuration containing a Simple Participant in domain 0 and a WAN Participant configured as server
    // (possibly also as client)
    RawConfiguration server_router_configuration =
            load_configuration_from_file(server_config_path);

    // Load configuration containing a Simple Participant in domain 1 and a WAN Participant configured as client
    // (possibly also as server)
    RawConfiguration client_router_configuration =
            load_configuration_from_file(client_config_path);

    // Create DDSRouter entity whose WAN Participant is configured as server
    DDSRouter server_router(server_router_configuration);
    server_router.start();

    // Create DDSRouter entity whose WAN Participant is configured as client
    DDSRouter client_router(client_router_configuration);
    client_router.start();

    // Wait for the endpoints to match before sending any data
    std::this_thread::sleep_for(std::chrono::seconds(5));

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

    client_router.stop();
    server_router.stop();
}

/**
 * Test communication between WAN participants by running \c test_WAN_communication for different configurations.
 * Different combinations of server/client configurations are tested.
 *
 * CASES:
 *  server <-> client
 *  server <-> server-client
 *  server-client <-> server-client
 */
void test_WAN_communication_all(std::string dir_path, bool basic_only=false)
{
    // server <-> client
    test_WAN_communication(dir_path + "server.yaml", dir_path + "client.yaml");

    // server <-> server-client
    test_WAN_communication(dir_path + "server.yaml", dir_path + "server-client-A.yaml");

    // This test is disabled for TCPv6 and TLSv6, as an underlying middleware issue resulting in no matching exists
    if (!basic_only)
    {
        // server-client <-> server-client
        test_WAN_communication(dir_path + "server-client-B.yaml", dir_path + "server-client-A.yaml");
    }
}

/**
 * Test whole DDSRouter initialization by initializing two Simple Participants
 */
TEST(DDSTest, simple_initialization)
{
    // Load configuration
    RawConfiguration router_configuration =
            load_configuration_from_file("resources/configurations/dds_test_simple_configuration.yaml");

    // Create DDSRouter entity
    DDSRouter router(router_configuration);

    // Let test finish without failing
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using a router with two Simple Participants at each domain.
 */
TEST(DDSTest, end_to_end_local_communication)
{
    test_local_communication("resources/configurations/dds_test_simple_configuration.yaml");
}

/**
 * Test communication in HelloWorldKeyed topic between two DDS participants created in different domains,
 * by using a router with two Simple Participants at each domain.
 */
TEST(DDSTest, end_to_end_local_communication_keyed)
{
    test_local_communication("resources/configurations/dds_test_simple_configuration.yaml", true);
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two WAN Participants connected
 * through UDPv4.
 */
TEST(DDSTest, end_to_end_WAN_communication_UDPv4)
{
    test_WAN_communication_all("resources/configurations/WAN/UDP/IPv4/");
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two WAN Participants connected
 * through UDPv6.
 */
TEST(DDSTest, end_to_end_WAN_communication_UDPv6)
{
    test_WAN_communication_all("resources/configurations/WAN/UDP/IPv6/");
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two WAN Participants connected
 * through TCPv4.
 */
TEST(DDSTest, end_to_end_WAN_communication_TCPv4)
{
    test_WAN_communication_all("resources/configurations/WAN/TCP/IPv4/");
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two WAN Participants connected
 * through TCPv6.
 */
TEST(DDSTest, end_to_end_WAN_communication_TCPv6)
{
    test_WAN_communication_all("resources/configurations/WAN/TCP/IPv6/", true);
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two WAN Participants connected
 * through TLSv4.
 */
TEST(DDSTest, end_to_end_WAN_communication_TLSv4)
{
    test_WAN_communication_all("resources/configurations/WAN/TLS/IPv4/");
    // test_WAN_communication("resources/configurations/WAN/TLS/IPv4/server-client-B.yaml", "resources/configurations/WAN/TLS/IPv4/server-client-A.yaml");
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two WAN Participants connected
 * through TLSv6.
 */
TEST(DDSTest, end_to_end_WAN_communication_TLSv6)
{
    test_WAN_communication_all("resources/configurations/WAN/TLS/IPv6/", true);
}

int main(
        int argc,
        char** argv)
{
    // Activate log
    Log::SetVerbosity(Log::Kind::Info);
    Log::SetCategoryFilter(std::regex("(DDSROUTER)"));

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
