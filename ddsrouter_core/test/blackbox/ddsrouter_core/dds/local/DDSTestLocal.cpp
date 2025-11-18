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
DdsRouterConfiguration dds_test_simple_configuration(
        bool disable_dynamic_discovery = false,
        bool transient_local_readers = false)
{
    DdsRouterConfiguration conf;

    // Always filter the test topics by topic name
    core::types::WildcardDdsFilterTopic topic;
    topic.topic_name.set_value(TOPIC_NAME);
    conf.ddspipe_configuration.allowlist.insert(
        utils::Heritable<core::types::WildcardDdsFilterTopic>::make_heritable(topic));

    if (disable_dynamic_discovery)
    {
        core::types::TopicQoS qos;
        if (transient_local_readers)
        {
            qos.reliability_qos = core::types::ReliabilityKind::RELIABLE;
            qos.durability_qos = core::types::DurabilityKind::TRANSIENT_LOCAL;
        }
        else
        {
            qos.reliability_qos = core::types::ReliabilityKind::BEST_EFFORT;
            qos.durability_qos = core::types::DurabilityKind::VOLATILE;
        }

        core::types::DdsTopic topic;
        topic.m_topic_name = TOPIC_NAME;
        topic.type_name = "HelloWorld";
        topic.topic_qos = qos;

        core::types::DdsTopic topic_keyed(topic);
        topic_keyed.type_name = "HelloWorldKeyed";
        topic_keyed.topic_qos.keyed = true;

        conf.ddspipe_configuration.builtin_topics.insert(utils::Heritable<core::types::DdsTopic>::make_heritable(topic));
        conf.ddspipe_configuration.builtin_topics.insert(utils::Heritable<core::types::DdsTopic>::make_heritable(
                    topic_keyed));
    }

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
 * Test communication between two DDS Participants hosted in the same device, but which are at different DDS domains.
 * This is accomplished by using a DDS Router instance with a Simple Participant deployed at each domain.
 *
 * The transient_local option changes the test behavior to verify that the communication is transient_local and all old data is sent
 * to Late Joiners.
 */
template <class MsgStruct, class MsgStructType>
void test_local_communication(
        DdsRouterConfiguration ddsrouter_configuration,
        uint32_t samples_to_receive = DEFAULT_SAMPLES_TO_RECEIVE,
        uint32_t time_between_samples = DEFAULT_MILLISECONDS_PUBLISH_LOOP,
        uint32_t msg_size = DEFAULT_MESSAGE_SIZE,
        bool transient_local = false)
{

    // Check there are no warnings/errors
    // TODO: Change threshold to \c Log::Kind::Warning once middleware warnings are solved
    // eprosima::ddsrouter::test::TestLogHandler test_log_handler(utils::Log::Kind::Error);
    INSTANTIATE_LOG_TESTER(eprosima::utils::Log::Kind::Error, 0, 0);

    uint32_t samples_sent = 0;
    std::atomic<uint32_t> samples_received(0);

    // Create a message with size specified by repeating the same string
    MsgStruct msg;

    MsgStructType type;

    std::string msg_str;

    // Add this string as many times as the msg size requires
    for (uint32_t i = 0; i < msg_size; i++)
    {
        msg_str += "Testing DdsRouter Blackbox Local Communication ...";
    }
    msg.message(msg_str);

    // Create DDS Publisher in domain 0
    TestPublisher<MsgStruct> publisher(type.is_compute_key_provided);

    ASSERT_TRUE(publisher.init(0));

    // Create DDS Subscriber in domain 1
    TestSubscriber<MsgStruct> subscriber(type.is_compute_key_provided, transient_local);

    ASSERT_TRUE(subscriber.init(1, &msg, &samples_received));

    // Create DdsRouter entity
    // The DDS Router does not start here in order to test a transient_local communication
    DdsRouter router(ddsrouter_configuration);

    if (transient_local)
    {
        // To check that the communication is transient_local and all previous published samples are sent to late joiner,
        // the publisher publish all data at once.
        for (samples_sent = 0; samples_sent < samples_to_receive; samples_sent++)
        {
            msg.index(samples_sent);
            ASSERT_EQ(publisher.publish(msg), eprosima::fastdds::dds::RETCODE_OK);

            // If time is 0 do not wait
            if (time_between_samples > 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(time_between_samples));
            }
        }

        // Once the publisher has already publish all its samples, the DDS Router starts.
        router.start();

        // The subscriber should receive all samples sent by the publisher when the communication was not
        // already established.
        while (samples_received.load() < samples_to_receive)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    else
    {
        // Start DDS Router
        router.start();

        // Start publishing
        while (samples_received.load() < samples_to_receive)
        {
            msg.index(++samples_sent);
            publisher.publish(msg);

            // If time is 0 do not wait
            if (time_between_samples > 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(time_between_samples));
            }
        }
    }

    router.stop();
}

/**
 * This test checks that, when the original writer parameter is not set by a writer,
 * the router still sets it to the original writer's GUID when forwarding the message.
 */
template <class MsgStruct, class MsgStructType>
void test_original_writer_forwarding_unset(
        DdsRouterConfiguration ddsrouter_configuration)
{
    INSTANTIATE_LOG_TESTER(eprosima::utils::Log::Kind::Error, 0, 0);

    uint32_t samples_sent = 0;
    std::atomic<uint32_t> samples_received(0);

    MsgStruct sent_msg;
    MsgStructType type;
    std::string msg_str;
    msg_str += "Testing DdsRouter Blackbox Local Communication ...";
    sent_msg.message(msg_str);
    // Create DDS Publisher in domain 0
    TestPublisher<MsgStruct> publisher(type.is_compute_key_provided);

    ASSERT_TRUE(publisher.init(0));

    // Create DDS Subscriber in domain 1
    TestSubscriber<MsgStruct> subscriber(type.is_compute_key_provided, true);
    ASSERT_TRUE(subscriber.init(1, &sent_msg, &samples_received));

    // Create DdsRouter entity
    DdsRouter router(ddsrouter_configuration);
    router.start();

    // CASE: Send message without original_writer_param, it should be set to the original writers guid
    // in the router either way
    sent_msg.index(++samples_sent);
    ASSERT_EQ(publisher.publish(sent_msg), eprosima::fastdds::dds::RETCODE_OK);
    // Watiting for the message to be received
    while (samples_received.load() < 1);

    ASSERT_EQ(subscriber.original_writer_guid(), publisher.original_writer_guid());

    router.stop();
}

/**
 * This test checks that, when the original writer parameter is populated by a writer and
 * is not equal to unknown, the router keeps that value when forwarding the message.
 */
template <class MsgStruct, class MsgStructType>
void test_original_writer_forwarding_populated(
        DdsRouterConfiguration ddsrouter_configuration)
{
    INSTANTIATE_LOG_TESTER(eprosima::utils::Log::Kind::Error, 0, 0);

    uint32_t samples_sent = 0;
    std::atomic<uint32_t> samples_received(0);

    MsgStruct sent_msg;
    MsgStructType type;
    std::string msg_str;
    msg_str += "Testing DdsRouter Blackbox Local Communication ...";
    sent_msg.message(msg_str);
    // Create DDS Publisher in domain 0
    TestPublisher<MsgStruct> publisher(type.is_compute_key_provided);

    ASSERT_TRUE(publisher.init(0));

    // Create DDS Subscriber in domain 1
    TestSubscriber<MsgStruct> subscriber(type.is_compute_key_provided, true);
    ASSERT_TRUE(subscriber.init(1, &sent_msg, &samples_received));

    // Create DdsRouter entity
    DdsRouter router(ddsrouter_configuration);
    router.start();

    // CASE: Send message with original_writer_param set to some value distinct to unknown, value must be kept
    sent_msg.index(++samples_sent);
    eprosima::fastdds::rtps::WriteParams params_with_og_writer;
    eprosima::fastdds::rtps::GUID_t guid({}, 0x12345678);
    params_with_og_writer.original_writer_info().original_writer_guid(guid);
    ASSERT_EQ(publisher.publish_with_params(sent_msg, params_with_og_writer), eprosima::fastdds::dds::RETCODE_OK);
    // Waiting for the message to be received
    while (samples_received.load() < 1)
    {
    }
    ASSERT_EQ(subscriber.original_writer_guid(), guid);

    router.stop();
}

/**
 * This test checks that, when the original writer parameter is populated by a writer and
 * is equal to unknown, the router populates it with the original writer's GUID when forwarding the message.
 */
template <class MsgStruct, class MsgStructType>
void test_original_writer_forwarding_unknown(
        DdsRouterConfiguration ddsrouter_configuration)
{
    INSTANTIATE_LOG_TESTER(eprosima::utils::Log::Kind::Error, 0, 0);

    uint32_t samples_sent = 0;
    std::atomic<uint32_t> samples_received(0);

    MsgStruct sent_msg;
    MsgStructType type;
    std::string msg_str;
    msg_str += "Testing DdsRouter Blackbox Local Communication ...";
    sent_msg.message(msg_str);
    // Create DDS Publisher in domain 0
    TestPublisher<MsgStruct> publisher(type.is_compute_key_provided);

    ASSERT_TRUE(publisher.init(0));

    // Create DDS Subscriber in domain 1
    TestSubscriber<MsgStruct> subscriber(type.is_compute_key_provided, true);
    ASSERT_TRUE(subscriber.init(1, &sent_msg, &samples_received));

    // Create DdsRouter entity
    DdsRouter router(ddsrouter_configuration);
    router.start();

    // CASE: Send message with original_writer_param set to unknown, should be set to other value
    sent_msg.index(++samples_sent);
    eprosima::fastdds::rtps::WriteParams params;
    params.original_writer_info(eprosima::fastdds::rtps::OriginalWriterInfo::unknown());
    ASSERT_EQ(publisher.publish_with_params(sent_msg, params), eprosima::fastdds::dds::RETCODE_OK);
    // Waiting for the message to be received
    while (samples_received.load() < 1)
    {
    }
    ASSERT_EQ(subscriber.original_writer_guid(), publisher.original_writer_guid());

    router.stop();
}

} /* namespace test */

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using a router with two Simple Participants at each domain.
 */
TEST(DDSTestLocal, end_to_end_local_communication)
{
    test::test_local_communication<HelloWorld, HelloWorldPubSubType>(
        test::dds_test_simple_configuration());
}

/**
 * Test communication in HelloWorldKeyed topic between two DDS participants created in different domains,
 * by using a router with two Simple Participants at each domain.
 */
TEST(DDSTestLocal, end_to_end_local_communication_keyed)
{
    test::test_local_communication<HelloWorldKeyed, HelloWorldKeyedPubSubType>(
        test::dds_test_simple_configuration());
}

/**
 * Test communication both in HelloWorld and HelloWorldKeyed topics between two DDS participants created in
 * different domains, by using a router with two Simple Participants at each domain.
 * In this test allowlist and blocklist are left empty, while only builtin topics are provided.
 */
TEST(DDSTestLocal, end_to_end_local_communication_disable_dynamic_discovery)
{
    test::test_local_communication<HelloWorld, HelloWorldPubSubType>(
        test::dds_test_simple_configuration(true));
}

TEST(DDSTestLocal, end_to_end_local_communication_disable_dynamic_discovery_keyed)
{
    test::test_local_communication<HelloWorldKeyed, HelloWorldKeyedPubSubType>(
        test::dds_test_simple_configuration(true));
}

/**
 * Test high frequency communication in HelloWorld topic between two DDS participants created in different domains,
 * by using a router with two Simple Participants at each domain.
 *
 * PARAMETERS:
 * - Frequency: max
 */
TEST(DDSTestLocal, end_to_end_local_communication_high_frequency)
{
    test::test_local_communication<HelloWorld, HelloWorldPubSubType>(
        test::dds_test_simple_configuration(),
        1000,       // wait for 1000 samples received
        0);         // send it without waiting from one sample to the other
}

/**
 * Test high message size communication in HelloWorld topic between two DDS participants created in different domains,
 * by using a router with two Simple Participants at each domain.
 *
 * PARAMETERS:
 * - Sample size: 500K
 */
TEST(DDSTestLocal, end_to_end_local_communication_high_size)
{
    test::test_local_communication<HelloWorld, HelloWorldPubSubType>(
        test::dds_test_simple_configuration(),
        test::DEFAULT_SAMPLES_TO_RECEIVE,
        test::DEFAULT_MILLISECONDS_PUBLISH_LOOP,
        10000);     // 500K message size
}

/**
 * Test high throughput communication in HelloWorld topic between two DDS participants created in different domains,
 * by using a router with two Simple Participants at each domain.
 *
 * PARAMETERS:
 * - Frequency: 1ms
 * - Sample size: 50K
 * -> Throughput: 50MBps
 */
TEST(DDSTestLocal, end_to_end_local_communication_high_throughput)
{
    test::test_local_communication<HelloWorld, HelloWorldPubSubType>(
        test::dds_test_simple_configuration(),
        500,
        1,
        1000);     // 50K message size
}

/**
 * Test transient_local communication in HelloWorld topic between two DDS participants created in different domains,
 * by using a router with two Simple Participants at each domain.
 */
TEST(DDSTestLocal, end_to_end_local_communication_transient_local)
{
    test::test_local_communication<HelloWorld, HelloWorldPubSubType>(
        test::dds_test_simple_configuration(),
        test::DEFAULT_SAMPLES_TO_RECEIVE,
        test::DEFAULT_MILLISECONDS_PUBLISH_LOOP,
        test::DEFAULT_MESSAGE_SIZE,
        true);
}

/**
 * Test transient_local communication in HelloWorld topic between two DDS participants created in different domains,
 * by using a router with two Simple Participants at each domain and using builtin-topics
 */
TEST(DDSTestLocal, end_to_end_local_communication_transient_local_disable_dynamic_discovery)
{
    test::test_local_communication<HelloWorld, HelloWorldPubSubType>(
        test::dds_test_simple_configuration(true, true),
        test::DEFAULT_SAMPLES_TO_RECEIVE,
        test::DEFAULT_MILLISECONDS_PUBLISH_LOOP,
        test::DEFAULT_MESSAGE_SIZE,
        true);
}

/**
 * This test checks that, when the original writer parameter is not set by a writer,
 * the router still sets it to the original writer's GUID when forwarding the message.
 */
TEST(DDSTestLocal, end_to_end_local_communication_original_writer_forwarding_unset)
{
    test::test_original_writer_forwarding_unset<HelloWorld, HelloWorldPubSubType>(
        test::dds_test_simple_configuration());
}

/**
 * This test checks that, when the original writer parameter is populated by a writer and
 * is not equal to unknown, the router keeps that value when forwarding the message.
 */
TEST(DDSTestLocal, end_to_end_local_communication_original_writer_forwarding_populated)
{
    test::test_original_writer_forwarding_populated<HelloWorld, HelloWorldPubSubType>(
        test::dds_test_simple_configuration());
}

/**
 * This test checks that, when the original writer parameter is populated by a writer and
 * is equal to unknown, the router populates it with the original writer's GUID when forwarding the message.
 */
TEST(DDSTestLocal, end_to_end_local_communication_original_writer_forwarding_unknown)
{
    test::test_original_writer_forwarding_unknown<HelloWorld, HelloWorldPubSubType>(
        test::dds_test_simple_configuration());
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
