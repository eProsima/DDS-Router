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
#include <thread>

#include <fastdds/dds/core/detail/DDSReturnCode.hpp>

#include <cpp_utils/Log.hpp>
#include <cpp_utils/testing/gtest_aux.hpp>
#include <cpp_utils/testing/LogChecker.hpp>

#include <ddspipe_core/types/topic/filter/WildcardDdsFilterTopic.hpp>

#include <ddsrouter_core/core/DdsRouter.hpp>

#include <test_participants.hpp>

#include <gtest/gtest.h>

using namespace eprosima;
using namespace eprosima::ddspipe;
using namespace eprosima::ddsrouter::core;

namespace test {

constexpr const uint32_t DEFAULT_SAMPLES_TO_RECEIVE = 5;
constexpr const uint32_t DEFAULT_MILLISECONDS_PUBLISH_LOOP = 100;
constexpr const uint32_t DEFAULT_SUBSCRIBER_MESSAGE_RECEPTION = 500;
constexpr const uint32_t DEFAULT_MESSAGE_SIZE = 1; // x50 bytes

/**
 * @brief Create a simple configuration for a DDS Router
 *
 * Create a configuration with 1 topic in its allowlist, forwarding data for all topics with name TOPIC_NAME
 * and any possible type (such as HelloWorld and HelloWorldKeyed) both with and without key.
 * Create 2 simple participants with domains 0 and 1
 *
 * @return DdsRouterConfiguration
 */
DdsRouterConfiguration dds_test_simple_configuration()
{
    DdsRouterConfiguration conf;

    // Always filter the test topics by topic name
    core::types::WildcardDdsFilterTopic topic;
    topic.topic_name.set_value(TOPIC_NAME);
    conf.ddspipe_configuration.allowlist.insert(
        utils::Heritable<core::types::WildcardDdsFilterTopic>::make_heritable(topic));

    // Two simple participants
    {
        auto part = std::make_shared<participants::SimpleParticipantConfiguration>();
        part->id = core::types::ParticipantId("participant_0");
        part->domain.domain_id = 0u;
        conf.participants_configurations.insert({types::ParticipantKind::simple, part});
    }

    {
        auto part = std::make_shared<participants::SimpleParticipantConfiguration>();
        part->id = core::types::ParticipantId("participant_1");
        part->domain.domain_id = 1u;
        conf.participants_configurations.insert({types::ParticipantKind::simple, part});
    }

    return conf;
}

/**
 * @brief Creates a configuration for a DDS Router with two XML participants at domains
 * 0 and 1 respectively. XML participants are DDS participants, thus they allow more
 * advanced QoS configuration through XML profiles.
 *
 * @return DdsRouterConfiguration
 */
DdsRouterConfiguration dds_test_dds_participants_config()
{
    DdsRouterConfiguration conf;

    // Always filter the test topics by topic name
    core::types::WildcardDdsFilterTopic topic;
    topic.topic_name.set_value(TOPIC_NAME);
    conf.ddspipe_configuration.allowlist.insert(
        utils::Heritable<core::types::WildcardDdsFilterTopic>::make_heritable(topic));

    // Create XML string with default QoS profile
    std::string xml_config =
        "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>"
        "<dds xmlns=\"http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles\">"
        "    <profiles>"
        "        <participant profile_name=\"participant_domain_0\">"
        "            <domainId>0</domainId>"
        "        </participant>"
        "        <participant profile_name=\"participant_domain_1\">"
        "            <domainId>1</domainId>"
        "        </participant>"
        "    </profiles>"
        "</dds>";

    // Load XML configuration into Fast DDS
    using namespace eprosima::fastdds::dds;
    if (RETCODE_OK != DomainParticipantFactory::get_instance()->load_XML_profiles_string(
        xml_config.c_str(), xml_config.length()))
    {
        throw std::runtime_error("Failed to load XML profiles");
    }

    // Two XML participants referencing the profile by name
    {
        auto part = std::make_shared<participants::XmlParticipantConfiguration>();
        part->id = core::types::ParticipantId("participant_0");
        part->participant_profile.set_value("participant_domain_0");
        conf.participants_configurations.insert({types::ParticipantKind::xml, part});
    }

    {
        auto part = std::make_shared<participants::XmlParticipantConfiguration>();
        part->id = core::types::ParticipantId("participant_1");
        part->participant_profile.set_value("participant_domain_1");
        conf.participants_configurations.insert({types::ParticipantKind::xml, part});
    }

    return conf;
}

/**
 * Test communication between two DDS Participants hosted in the same device, but which are at different DDS domains.
 * This is accomplished by using a DDS Router instance with a Simple Participant deployed at each domain.
 *
 * The reliable option changes the test behavior to verify that the communication is reliable and all old data is sent
 * to Late Joiners.
 */
void test_local_communication_key_dispose(
        DdsRouterConfiguration ddsrouter_configuration,
        uint32_t samples_to_receive = DEFAULT_SAMPLES_TO_RECEIVE,
        uint32_t time_between_samples = DEFAULT_MILLISECONDS_PUBLISH_LOOP,
        uint32_t msg_size = DEFAULT_MESSAGE_SIZE)
{
    // Check there are no warnings/errors
    // TODO: Change threshold to \c Log::Kind::Warning once middleware warnings are solved
    // eprosima::ddsrouter::test::TestLogHandler test_log_handler(utils::Log::Kind::Error);
    INSTANTIATE_LOG_TESTER(eprosima::utils::Log::Kind::Error, 0, 0);

    uint32_t samples_sent = 0;
    std::atomic<uint32_t> samples_received(0);

    // Create a message with size specified by repeating the same string
    HelloWorldKeyed msg;

    HelloWorldKeyedPubSubType type;

    std::string msg_str;

    // Add this string as many times as the msg size requires
    for (uint32_t i = 0; i < msg_size; i++)
    {
        msg_str += "Testing DdsRouter Blackbox Local Communication ...";
    }
    msg.message(msg_str);
    msg.id(666);

    // Create DDS Publisher in domain 0
    TestPublisher<HelloWorldKeyed> publisher(type.is_compute_key_provided);

    ASSERT_TRUE(publisher.init(0));

    // Create DDS Subscriber in domain 1
    TestSubscriber<HelloWorldKeyed> subscriber(type.is_compute_key_provided);

    ASSERT_TRUE(subscriber.init(1, &msg, &samples_received));

    // Create DdsRouter entity
    // The DDS Router does not start here in order to test a reliable communication
    DdsRouter router(ddsrouter_configuration);

    // Start DDS Router
    router.start();

    // Start publishing
    while (samples_received.load() < samples_to_receive)
    {
        msg.index(++samples_sent);
        ASSERT_EQ(publisher.publish(msg), fastdds::dds::RETCODE_OK) << samples_sent;

        // If time is 0 do not wait
        if (time_between_samples > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(time_between_samples));
        }
    }

    // All samples received, now dispose key from publisher and check that subscriber has receive it
    ASSERT_TRUE(publisher.dispose_key(msg) == fastdds::dds::RETCODE_OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(DEFAULT_SUBSCRIBER_MESSAGE_RECEPTION));
    samples_received.store(0);

    // Keep publishing
    while (samples_received.load() < samples_to_receive)
    {
        msg.index(++samples_sent);
        ASSERT_EQ(publisher.publish(msg), fastdds::dds::RETCODE_OK) << samples_sent;

        // If time is 0 do not wait
        if (time_between_samples > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(time_between_samples));
        }
    }

    // All samples received, now dispose key from publisher and check that subscriber has receive it
    ASSERT_TRUE(publisher.dispose_key(msg) == fastdds::dds::RETCODE_OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(DEFAULT_SUBSCRIBER_MESSAGE_RECEPTION));

    ASSERT_EQ(2u, subscriber.n_disposed());

    router.stop();
}


/**
 * Test communication between two DDS Participants hosted in the same device, but which are at different DDS domains.
 * This is accomplished by using a DDS Router instance with a Simple Participant deployed at each domain.
 *
 * The reliable option changes the test behavior to verify that the communication is reliable and all old data is sent
 * to Late Joiners.
 *
 * However, in this test the disposed key messages are sent without payload.
 */
void test_local_communication_key_dispose_without_payload(
        DdsRouterConfiguration ddsrouter_configuration,
        uint32_t samples_to_receive = DEFAULT_SAMPLES_TO_RECEIVE,
        uint32_t time_between_samples = DEFAULT_MILLISECONDS_PUBLISH_LOOP,
        uint32_t msg_size = DEFAULT_MESSAGE_SIZE)
{
    // Check there are no warnings/errors
    // TODO: Change threshold to \c Log::Kind::Warning once middleware warnings are solved
    // eprosima::ddsrouter::test::TestLogHandler test_log_handler(utils::Log::Kind::Error);
    INSTANTIATE_LOG_TESTER(eprosima::utils::Log::Kind::Error, 0, 0);

    uint32_t samples_sent = 0;
    std::atomic<uint32_t> samples_received(0);

    // Create a message with size specified by repeating the same string
    HelloWorldKeyed msg;
    HelloWorldKeyedZeroSizePayloadPubSubType type;

    std::string msg_str;

    // Add this string as many times as the msg size requires
    for (uint32_t i = 0; i < msg_size; i++)
    {
        msg_str += "Testing DdsRouter Blackbox Local Communication ...";
    }
    msg.message(msg_str);
    msg.id(666);

    // Create DDS Publisher in domain 0
    TestPublisher<HelloWorldKeyed> publisher(type.is_compute_key_provided);

    ASSERT_TRUE(publisher.init(0));

    // Create DDS Subscriber in domain 1
    TestSubscriber<HelloWorldKeyed> subscriber(type.is_compute_key_provided);

    ASSERT_TRUE(subscriber.init(1, &msg, &samples_received));

    // Create DdsRouter entity
    // The DDS Router does not start here in order to test a reliable communication
    DdsRouter router(ddsrouter_configuration);

    // Start DDS Router
    router.start();

    // Start publishing
    while (samples_received.load() < samples_to_receive)
    {
        msg.index(++samples_sent);
        ASSERT_EQ(publisher.publish(msg), fastdds::dds::RETCODE_OK) << samples_sent;

        // If time is 0 do not wait
        if (time_between_samples > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(time_between_samples));
        }
    }

    // All samples received, now dispose key from publisher and check that subscriber has receive it
    ASSERT_TRUE(publisher.dispose_key(msg) == fastdds::dds::RETCODE_OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(DEFAULT_SUBSCRIBER_MESSAGE_RECEPTION));
    samples_received.store(0);

    // Keep publishing
    while (samples_received.load() < samples_to_receive)
    {
        msg.index(++samples_sent);
        ASSERT_EQ(publisher.publish(msg), fastdds::dds::RETCODE_OK) << samples_sent;

        // If time is 0 do not wait
        if (time_between_samples > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(time_between_samples));
        }
    }

    // All samples received, now dispose key from publisher and check that subscriber has receive it
    ASSERT_TRUE(publisher.dispose_key(msg) == fastdds::dds::RETCODE_OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(DEFAULT_SUBSCRIBER_MESSAGE_RECEPTION));

    ASSERT_EQ(2u, subscriber.n_disposed());

    router.stop();
}

void test_local_communication_unregister_dispose(
        DdsRouterConfiguration ddsrouter_configuration,
        uint32_t samples_to_receive = DEFAULT_SAMPLES_TO_RECEIVE,
        uint32_t time_between_samples = DEFAULT_MILLISECONDS_PUBLISH_LOOP,
        uint32_t msg_size = DEFAULT_MESSAGE_SIZE)
{
    // Check there are no warnings/errors
    // TODO: Change threshold to \c Log::Kind::Warning once middleware warnings are solved
    // eprosima::ddsrouter::test::TestLogHandler test_log_handler(utils::Log::Kind::Error);
    INSTANTIATE_LOG_TESTER(eprosima::utils::Log::Kind::Error, 0, 0);

    uint32_t samples_sent = 0;
    std::atomic<uint32_t> samples_received(0);

    // Create a message with size specified by repeating the same string
    HelloWorldKeyed msg;
    HelloWorldKeyedPubSubType type;

    std::string msg_str;

    // Add this string as many times as the msg size requires
    for (uint32_t i = 0; i < msg_size; i++)
    {
        msg_str += "Testing DdsRouter Blackbox Local Communication ...";
    }
    msg.message(msg_str);
    msg.id(666);

    // Create DDS Publisher in domain 0
    TestPublisher<HelloWorldKeyed> publisher(type.is_compute_key_provided);

    ASSERT_TRUE(publisher.init(0));

    // Create DDS Subscriber in domain 1
    TestSubscriber<HelloWorldKeyed> subscriber(type.is_compute_key_provided);

    ASSERT_TRUE(subscriber.init(1, &msg, &samples_received));
    // Initial condition: there have not been any not_alive_no_writers events
    ASSERT_EQ(0u, subscriber.n_disposed());

    // Create DdsRouter entity
    // The DDS Router does not start here in order to test a reliable communication
    DdsRouter router(ddsrouter_configuration);

    // Start DDS Router
    router.start();
    // Start publishing
    while (samples_received.load() < samples_to_receive)
    {
        msg.index(++samples_sent);
        ASSERT_EQ(publisher.publish(msg), fastdds::dds::RETCODE_OK) << samples_sent;

        // If time is 0 do not wait
        if (time_between_samples > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(time_between_samples));
        }
    }
    // All samples received, now unregister key from publisher and check that subscriber has received
    // a not alive no writers event
    ASSERT_TRUE(publisher.unregister_key(msg) == fastdds::dds::RETCODE_OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(DEFAULT_SUBSCRIBER_MESSAGE_RECEPTION));

    // As the default change kind of an unregistered instance is NOT_ALIVE_DISPOSED_UNREGISTERED
    // and the transition to the state NOT_ALIVE_DISPOSED has priority over NOT_ALIVE_NO_WRITERS,
    // we check for disposed instances instead of no writers
    ASSERT_EQ(1u, subscriber.n_disposed());

    router.stop();
}

void test_local_communication_unregister_dispose_zero_payload(
        DdsRouterConfiguration ddsrouter_configuration,
        uint32_t samples_to_receive = DEFAULT_SAMPLES_TO_RECEIVE,
        uint32_t time_between_samples = DEFAULT_MILLISECONDS_PUBLISH_LOOP,
        uint32_t msg_size = DEFAULT_MESSAGE_SIZE)
{
    // Check there are no warnings/errors
    // TODO: Change threshold to \c Log::Kind::Warning once middleware warnings are solved
    // eprosima::ddsrouter::test::TestLogHandler test_log_handler(utils::Log::Kind::Error);
    INSTANTIATE_LOG_TESTER(eprosima::utils::Log::Kind::Error, 0, 0);

    uint32_t samples_sent = 0;
    std::atomic<uint32_t> samples_received(0);

    // Create a message with size specified by repeating the same string
    HelloWorldKeyed msg;
    HelloWorldKeyedPubSubType type;

    std::string msg_str;

    // Add this string as many times as the msg size requires
    for (uint32_t i = 0; i < msg_size; i++)
    {
        msg_str += "Testing DdsRouter Blackbox Local Communication ...";
    }
    msg.message(msg_str);
    msg.id(666);

    // Create DDS Publisher in domain 0
    TestPublisher<HelloWorldKeyed> publisher(type.is_compute_key_provided);

    ASSERT_TRUE(publisher.init(0));

    // Create DDS Subscriber in domain 1
    TestSubscriber<HelloWorldKeyed> subscriber(type.is_compute_key_provided);

    ASSERT_TRUE(subscriber.init(1, &msg, &samples_received));
    // Initial condition: there have not been any not_alive_no_writers events
    ASSERT_EQ(0u, subscriber.n_disposed());

    // Create DdsRouter entity
    // The DDS Router does not start here in order to test a reliable communication
    DdsRouter router(ddsrouter_configuration);

    // Start DDS Router
    router.start();
    // Start publishing
    while (samples_received.load() < samples_to_receive)
    {
        msg.index(++samples_sent);
        ASSERT_EQ(publisher.publish(msg), fastdds::dds::RETCODE_OK) << samples_sent;

        // If time is 0 do not wait
        if (time_between_samples > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(time_between_samples));
        }
    }
    // All samples received, now unregister key from publisher and check that subscriber has received
    // a not alive no writers event
    ASSERT_TRUE(publisher.unregister_key(msg) == fastdds::dds::RETCODE_OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(DEFAULT_SUBSCRIBER_MESSAGE_RECEPTION));

    // As the default change kind of an unregistered instance is NOT_ALIVE_DISPOSED_UNREGISTERED
    // and the transition to the state NOT_ALIVE_DISPOSED has priority over NOT_ALIVE_NO_WRITERS,
    // we check for disposed instances instead of no writers
    ASSERT_EQ(1u, subscriber.n_disposed());

    router.stop();
}

void test_local_communication_unregister(
        DdsRouterConfiguration ddsrouter_configuration,
        uint32_t samples_to_receive = DEFAULT_SAMPLES_TO_RECEIVE,
        uint32_t time_between_samples = DEFAULT_MILLISECONDS_PUBLISH_LOOP,
        uint32_t msg_size = DEFAULT_MESSAGE_SIZE)
{
    // Check there are no warnings/errors
    // TODO: Change threshold to \c Log::Kind::Warning once middleware warnings are solved
    // eprosima::ddsrouter::test::TestLogHandler test_log_handler(utils::Log::Kind::Error);
    INSTANTIATE_LOG_TESTER(eprosima::utils::Log::Kind::Error, 0, 0);

    uint32_t samples_sent = 0;
    std::atomic<uint32_t> samples_received(0);

    // Create a message with size specified by repeating the same string
    HelloWorldKeyed msg;
    HelloWorldKeyedPubSubType type;

    std::string msg_str;

    // Add this string as many times as the msg size requires
    for (uint32_t i = 0; i < msg_size; i++)
    {
        msg_str += "Testing DdsRouter Blackbox Local Communication ...";
    }
    msg.message(msg_str);
    msg.id(666);

    // Create DDS Publisher in domain 0
    TestPublisher<HelloWorldKeyed> publisher(type.is_compute_key_provided);

    eprosima::fastdds::dds::DomainParticipantQos pqos = eprosima::fastdds::dds::DomainParticipantQos();
    eprosima::fastdds::dds::DataWriterQos wqos = eprosima::fastdds::dds::DataWriterQos();
    // Necessary to receive only an unresgiter and not a dispose, forcing transition to not_alive_no_writers
    wqos.writer_data_lifecycle().autodispose_unregistered_instances = false;
    eprosima::fastdds::dds::PublisherQos pubqos = eprosima::fastdds::dds::PublisherQos();
    eprosima::fastdds::dds::TopicQos tqos = eprosima::fastdds::dds::TopicQos();
    ASSERT_TRUE(publisher.init_with_custom_qos(0, pqos, wqos, pubqos, tqos));

    // Create DDS Subscriber in domain 1
    TestSubscriber<HelloWorldKeyed> subscriber(type.is_compute_key_provided);

    ASSERT_TRUE(subscriber.init(1, &msg, &samples_received));
    // Initial condition: there have not been any not_alive_no_writers events
    ASSERT_EQ(0u, subscriber.n_key_no_writers());

    // Create DdsRouter entity
    // The DDS Router does not start here in order to test a reliable communication
    DdsRouter router(ddsrouter_configuration);

    // Start DDS Router
    router.start();
    // Start publishing
    while (samples_received.load() < samples_to_receive)
    {
        msg.index(++samples_sent);
        ASSERT_EQ(publisher.publish(msg), fastdds::dds::RETCODE_OK) << samples_sent;

        // If time is 0 do not wait
        if (time_between_samples > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(time_between_samples));
        }
    }
    // All samples received, now unregister key from publisher and check that subscriber has received
    // a not alive no writers event
    ASSERT_TRUE(publisher.unregister_key(msg) == fastdds::dds::RETCODE_OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(DEFAULT_SUBSCRIBER_MESSAGE_RECEPTION));

    // Expectation: exactly one NOT_ALIVE_NO_WRITERS event was received as a consequence of the unregister
    ASSERT_EQ(1u, subscriber.n_key_no_writers());

    router.stop();
}

void test_local_communication_unregister_zero_payload(
        DdsRouterConfiguration ddsrouter_configuration,
        uint32_t samples_to_receive = DEFAULT_SAMPLES_TO_RECEIVE,
        uint32_t time_between_samples = DEFAULT_MILLISECONDS_PUBLISH_LOOP,
        uint32_t msg_size = DEFAULT_MESSAGE_SIZE)
{
    // Check there are no warnings/errors
    // TODO: Change threshold to \c Log::Kind::Warning once middleware warnings are solved
    // eprosima::ddsrouter::test::TestLogHandler test_log_handler(utils::Log::Kind::Error);
    INSTANTIATE_LOG_TESTER(eprosima::utils::Log::Kind::Error, 0, 0);

    uint32_t samples_sent = 0;
    std::atomic<uint32_t> samples_received(0);

    // Create a message with size specified by repeating the same string
    HelloWorldKeyed msg;
    HelloWorldKeyedZeroSizePayloadPubSubType type;

    std::string msg_str;

    // Add this string as many times as the msg size requires
    for (uint32_t i = 0; i < msg_size; i++)
    {
        msg_str += "Testing DdsRouter Blackbox Local Communication ...";
    }
    msg.message(msg_str);
    msg.id(666);

    // Create DDS Publisher in domain 0
    TestPublisher<HelloWorldKeyed> publisher(type.is_compute_key_provided);

    eprosima::fastdds::dds::DomainParticipantQos pqos = eprosima::fastdds::dds::DomainParticipantQos();
    eprosima::fastdds::dds::DataWriterQos wqos = eprosima::fastdds::dds::DataWriterQos();
    // Necessary to receive only an unresgiter and not a dispose, forcing transition to not_alive_no_writers
    wqos.writer_data_lifecycle().autodispose_unregistered_instances = false;
    eprosima::fastdds::dds::PublisherQos pubqos = eprosima::fastdds::dds::PublisherQos();
    eprosima::fastdds::dds::TopicQos tqos = eprosima::fastdds::dds::TopicQos();
    ASSERT_TRUE(publisher.init_with_custom_qos(0, pqos, wqos, pubqos, tqos));

    // Create DDS Subscriber in domain 1
    TestSubscriber<HelloWorldKeyed> subscriber(type.is_compute_key_provided);

    ASSERT_TRUE(subscriber.init(1, &msg, &samples_received));
    // Initial condition: there have not been any not_alive_no_writers events
    ASSERT_EQ(0u, subscriber.n_key_no_writers());

    // Create DdsRouter entity
    // The DDS Router does not start here in order to test a reliable communication
    DdsRouter router(ddsrouter_configuration);

    // Start DDS Router
    router.start();
    // Start publishing
    while (samples_received.load() < samples_to_receive)
    {
        msg.index(++samples_sent);
        ASSERT_EQ(publisher.publish(msg), fastdds::dds::RETCODE_OK) << samples_sent;

        // If time is 0 do not wait
        if (time_between_samples > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(time_between_samples));
        }
    }
    // All samples received, now unregister key from publisher and check that subscriber has received
    // a not alive no writers event
    ASSERT_TRUE(publisher.unregister_key(msg) == fastdds::dds::RETCODE_OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(DEFAULT_SUBSCRIBER_MESSAGE_RECEPTION));

    // Expectation: exactly one NOT_ALIVE_NO_WRITERS event was received as a consequence of the unregister
    ASSERT_EQ(1u, subscriber.n_key_no_writers());

    router.stop();
}

/**
 * Tests that the lease duration mechanism triggers a not alive no writers event even on readers connected
 * to a writer through a DDS Router instance. In this case, the publisher is created with a very low lease
 * duration in order to trigger the event fast enough.
 */
void test_local_communication_lease_duration_triggers_not_alive_no_writers(
        DdsRouterConfiguration ddsrouter_configuration,
        uint32_t writer_lease_duration_ns,
        uint32_t samples_to_receive = DEFAULT_SAMPLES_TO_RECEIVE,
        uint32_t time_between_samples = DEFAULT_MILLISECONDS_PUBLISH_LOOP,
        uint32_t msg_size = DEFAULT_MESSAGE_SIZE)
{
    INSTANTIATE_LOG_TESTER(eprosima::utils::Log::Kind::Error, 0, 0);

    uint32_t samples_sent = 0;
    std::atomic<uint32_t> samples_received(0);
    uint32_t pub_lease_duration_seconds = 0;
    uint32_t pub_lease_duration_nanoseconds = writer_lease_duration_ns;

    // Create a message with size specified by repeating the same string
    HelloWorldKeyed msg;
    HelloWorldKeyedPubSubType type;

    std::string msg_str;

    // Add this string as many times as the msg size requires
    for (uint32_t i = 0; i < msg_size; i++)
    {
        msg_str += "Testing DdsRouter Blackbox Local Communication ...";
    }
    msg.message(msg_str);
    msg.id(666);

    // Create DDS Publisher in domain 0
    TestPublisher<HelloWorldKeyed> publisher(type.is_compute_key_provided);

    eprosima::fastdds::dds::DomainParticipantQos pqos = eprosima::fastdds::dds::DomainParticipantQos();
    // Lowering lease duration to trigger not alive no writers faster in the subscriber
    pqos.wire_protocol().builtin.discovery_config.leaseDuration = Duration_t(pub_lease_duration_seconds, pub_lease_duration_nanoseconds);
    pqos.wire_protocol().builtin.discovery_config.leaseDuration_announcementperiod = Duration_t(pub_lease_duration_seconds, pub_lease_duration_nanoseconds-1);
    eprosima::fastdds::dds::DataWriterQos wqos = eprosima::fastdds::dds::DataWriterQos();
    // Necessary to receive only an unregister and not a dispose, forcing transition to not_alive_no_writers
    wqos.writer_data_lifecycle().autodispose_unregistered_instances = false;
    eprosima::fastdds::dds::PublisherQos pubqos = eprosima::fastdds::dds::PublisherQos();
    eprosima::fastdds::dds::TopicQos tqos = eprosima::fastdds::dds::TopicQos();
    ASSERT_TRUE(publisher.init_with_custom_qos(0, pqos, wqos, pubqos, tqos));

    // Create DDS Subscriber in domain 1
    TestSubscriber<HelloWorldKeyed> subscriber(type.is_compute_key_provided);

    ASSERT_TRUE(subscriber.init(1, &msg, &samples_received));
    // Initial condition: there have not been any not_alive_no_writers events
    ASSERT_EQ(0u, subscriber.n_key_no_writers());

    // Create DdsRouter entity
    // The DDS Router does not start here in order to test a reliable communication
    DdsRouter router(ddsrouter_configuration);
    // Start DDS Router
    router.start();
    // Start publishing
    while (samples_received.load() < samples_to_receive)
    {
        msg.index(++samples_sent);
        ASSERT_EQ(publisher.publish(msg), fastdds::dds::RETCODE_OK) << samples_sent;
    }

    // All samples received, now wait for lease duration to check that the reader has received a
    // NOT_ALIVE_NO_WRITERS event from the take method
    std::this_thread::sleep_for(std::chrono::nanoseconds(static_cast<int>(writer_lease_duration_ns * 2)));
    ASSERT_EQ(1u, subscriber.n_key_no_writers());

    router.stop();
}

/**
 * Tests that the lease duration mechanism triggers a not alive no writers event even on readers connected
 * to a writer through a DDS Router instance. In this case, the publisher is created with a very low lease
 * duration and is destroyed early in order to trigger the event fast enough.
 */
void test_local_communication_dead_writer_triggers_not_alive_no_writers(
        DdsRouterConfiguration ddsrouter_configuration,
        uint32_t writer_lease_duration_ns,
        uint32_t samples_to_receive = DEFAULT_SAMPLES_TO_RECEIVE,
        uint32_t time_between_samples = DEFAULT_MILLISECONDS_PUBLISH_LOOP,
        uint32_t msg_size = DEFAULT_MESSAGE_SIZE)
{
    INSTANTIATE_LOG_TESTER(eprosima::utils::Log::Kind::Error, 0, 0);

    uint32_t samples_sent = 0;
    std::atomic<uint32_t> samples_received(0);
    uint32_t pub_lease_duration_seconds = 0;
    uint32_t pub_lease_duration_nanoseconds = writer_lease_duration_ns;

    // Create a message with size specified by repeating the same string
    HelloWorldKeyed msg;
    HelloWorldKeyedPubSubType type;

    std::string msg_str;

    // Add this string as many times as the msg size requires
    for (uint32_t i = 0; i < msg_size; i++)
    {
        msg_str += "Testing DdsRouter Blackbox Local Communication ...";
    }
    msg.message(msg_str);
    msg.id(666);

    // Create DDS Subscriber in domain 1
    TestSubscriber<HelloWorldKeyed> subscriber(type.is_compute_key_provided);
    // Create DdsRouter entity
    DdsRouter router(ddsrouter_configuration);
    {
        // Create DDS Publisher in domain 0
        TestPublisher<HelloWorldKeyed> publisher(type.is_compute_key_provided);

        eprosima::fastdds::dds::DomainParticipantQos pqos = eprosima::fastdds::dds::DomainParticipantQos();
        // Lowering lease duration to trigger not alive no writers faster in the subscriber
        pqos.wire_protocol().builtin.discovery_config.leaseDuration = Duration_t(pub_lease_duration_seconds, pub_lease_duration_nanoseconds);
        pqos.wire_protocol().builtin.discovery_config.leaseDuration_announcementperiod = Duration_t(pub_lease_duration_seconds, pub_lease_duration_nanoseconds-1);
        eprosima::fastdds::dds::DataWriterQos wqos = eprosima::fastdds::dds::DataWriterQos();
        // Necessary to receive only an unregister and not a dispose, forcing transition to not_alive_no_writers
        wqos.writer_data_lifecycle().autodispose_unregistered_instances = false;
        eprosima::fastdds::dds::PublisherQos pubqos = eprosima::fastdds::dds::PublisherQos();
        eprosima::fastdds::dds::TopicQos tqos = eprosima::fastdds::dds::TopicQos();
        ASSERT_TRUE(publisher.init_with_custom_qos(0, pqos, wqos, pubqos, tqos));
        ASSERT_TRUE(subscriber.init(1, &msg, &samples_received));
        // Initial condition: there have not been any not_alive_no_writers events
        ASSERT_EQ(0u, subscriber.n_key_no_writers());
        // Start DDS Router
        router.start();
        // Start publishing
        while (samples_received.load() < samples_to_receive)
        {
            msg.index(++samples_sent);
            ASSERT_EQ(publisher.publish(msg), fastdds::dds::RETCODE_OK) << samples_sent;
        }
    } // Publisher goes out of scope and is destroyed here

    // All samples received, now wait for lease duration to check that the writer receives
    // a NOT_ALIVE_NO_WRITERS event from the take method even if the publisher is destroyed
    std::this_thread::sleep_for(std::chrono::nanoseconds(static_cast<int>(writer_lease_duration_ns * 2)));
    ASSERT_EQ(1u, subscriber.n_key_no_writers());

    router.stop();
}

} /* namespace test */

/**
 * Test that dispose values from the publisher are correctly received by the subscriber from the router.
 */
TEST(DDSTestLocalInstances, end_to_end_local_communication_key_dispose)
{
    test::test_local_communication_key_dispose(
        test::dds_test_simple_configuration());
}

TEST(DDSTestLocalInstances, end_to_end_local_communication_key_dispose_without_payload)
{
    test::test_local_communication_key_dispose_without_payload(
        test::dds_test_simple_configuration());
}

TEST(DDSTestLocalInstances, end_to_end_local_communication_unregister_dispose)
{
    test::test_local_communication_unregister_dispose(
        test::dds_test_simple_configuration());
}

TEST(DDSTestLocalInstances, end_to_end_local_communication_unregister_dispose_zero_payload)
{
    test::test_local_communication_unregister_dispose_zero_payload(
        test::dds_test_simple_configuration());
}

TEST(DDSTestLocalInstances, end_to_end_local_communication_unregister)
{
    test::test_local_communication_unregister(
        test::dds_test_simple_configuration());
}

TEST(DDSTestLocalInstances, end_to_end_local_communication_unregister_zero_payload)
{
    test::test_local_communication_unregister_zero_payload(
        test::dds_test_simple_configuration());
}

// For this to work, DDS participants are required
TEST(DDSTestLocalInstances, end_to_end_local_communication_not_alive_no_writers_triggered_by_writer_lease_duration)
{
    // Router lease duration is default
    int writer_lease_duration_ns = 500000000; // 0.5 seconds
    test::test_local_communication_lease_duration_triggers_not_alive_no_writers(
        test::dds_test_dds_participants_config(), writer_lease_duration_ns);
}

// For this to work, DDS participants are required
TEST(DDSTestLocalInstances, end_to_end_local_communication_not_alive_no_writers_triggered_by_dead_writer)
{
    // Router lease duration is default
    int writer_lease_duration_ns = 500000000; //  0.5 seconds
    test::test_local_communication_dead_writer_triggers_not_alive_no_writers(
         test::dds_test_dds_participants_config(), writer_lease_duration_ns);
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    eprosima::fastdds::dds::Log::SetVerbosity(eprosima::fastdds::dds::Log::Kind::Info);


    return RUN_ALL_TESTS();
}
