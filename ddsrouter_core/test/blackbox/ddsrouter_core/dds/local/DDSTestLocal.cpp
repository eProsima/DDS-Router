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

#include <gtest_aux.hpp>
#include <gtest/gtest.h>
#include <TestLogHandler.hpp>

#include <ddsrouter_core/core/DDSRouter.hpp>
#include <ddsrouter_core/types/topic/filter/WildcardDdsFilterTopic.hpp>
#include <ddsrouter_utils/Log.hpp>

#include <test_participants.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
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
 * @return configuration::DDSRouterConfiguration
 */
configuration::DDSRouterConfiguration dds_test_simple_configuration(
        bool disable_dynamic_discovery = false,
        bool transient_local_readers = false)
{
    // Always filter the test topics by topic name
    std::set<std::shared_ptr<types::DdsFilterTopic>> allowlist;   // empty
    allowlist.insert(std::make_shared<types::WildcardDdsFilterTopic>(TOPIC_NAME));

    std::set<std::shared_ptr<types::DdsFilterTopic>> blocklist;   // empty

    std::set<std::shared_ptr<types::DdsTopic>> builtin_topics;   // empty

    if (disable_dynamic_discovery)
    {
        types::TopicQoS qos;
        if (transient_local_readers)
        {
            qos.reliability_qos = types::ReliabilityKind::RELIABLE;
            qos.durability_qos = types::DurabilityKind::TRANSIENT_LOCAL;
        }
        else
        {
            qos.reliability_qos = types::ReliabilityKind::BEST_EFFORT;
            qos.durability_qos = types::DurabilityKind::VOLATILE;
        }

        builtin_topics.insert(
            std::make_shared<types::DdsTopic>(TOPIC_NAME, "HelloWorld", false, qos));
        builtin_topics.insert(
            std::make_shared<types::DdsTopic>(TOPIC_NAME, "HelloWorldKeyed", true, qos));
    }

    // Two simple participants
    std::set<std::shared_ptr<configuration::ParticipantConfiguration>> participants_configurations(
                    {
                        std::make_shared<configuration::SimpleParticipantConfiguration>(
                            types::ParticipantId("participant_0"),
                            types::ParticipantKind(types::ParticipantKind::simple_rtps),
                            false,
                            types::DomainId(0u)
                            ),
                        std::make_shared<configuration::SimpleParticipantConfiguration>(
                            types::ParticipantId("participant_1"),
                            types::ParticipantKind(types::ParticipantKind::simple_rtps),
                            false,
                            types::DomainId(1u)
                            )
                    }
        );

    return configuration::DDSRouterConfiguration(
        allowlist,
        blocklist,
        builtin_topics,
        participants_configurations,
        configuration::SpecsConfiguration());
}

/**
 * Test communication between two DDS Participants hosted in the same device, but which are at different DDS domains.
 * This is accomplished by using a DDS Router instance with a Simple Participant deployed at each domain.
 *
 * The transient_local option changes the test behavior to verify that the communication is transient_local and all old data is sent
 * to Late Joiners.
 */
template <class MsgStruct>
void test_local_communication(
        configuration::DDSRouterConfiguration ddsrouter_configuration,
        uint32_t samples_to_receive = DEFAULT_SAMPLES_TO_RECEIVE,
        uint32_t time_between_samples = DEFAULT_MILLISECONDS_PUBLISH_LOOP,
        uint32_t msg_size = DEFAULT_MESSAGE_SIZE,
        bool transient_local = false)
{

    // Check there are no warnings/errors
    // TODO: Change threshold to \c Log::Kind::Warning once middleware warnings are solved
    eprosima::ddsrouter::test::TestLogHandler test_log_handler(utils::Log::Kind::Error);

    uint32_t samples_sent = 0;
    std::atomic<uint32_t> samples_received(0);

    // Create a message with size specified by repeating the same string
    MsgStruct msg;
    std::string msg_str;

    // Add this string as many times as the msg size requires
    for (uint32_t i = 0; i < msg_size; i++)
    {
        msg_str += "Testing DDSRouter Blackbox Local Communication ...";
    }
    msg.message(msg_str);

    // Create DDS Publisher in domain 0
    TestPublisher<MsgStruct> publisher(msg.isKeyDefined());
    ASSERT_TRUE(publisher.init(0));

    // Create DDS Subscriber in domain 1
    TestSubscriber<MsgStruct> subscriber(msg.isKeyDefined(), transient_local);
    ASSERT_TRUE(subscriber.init(1, &msg, &samples_received));

    // Create DDSRouter entity
    // The DDS Router does not start here in order to test a transient_local communication
    DDSRouter router(ddsrouter_configuration);

    if (transient_local)
    {
        // To check that the communication is transient_local and all previous published samples are sent to late joiner,
        // the publisher publish all data at once.
        for (samples_sent = 0; samples_sent < samples_to_receive; samples_sent++)
        {
            msg.index(samples_sent);
            ASSERT_TRUE(publisher.publish(msg));

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

} /* namespace test */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

using namespace eprosima::ddsrouter::core;
using namespace eprosima::ddsrouter::core::types;

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using a router with two Simple Participants at each domain.
 */
TEST(DDSTestLocal, end_to_end_local_communication)
{
    test::test_local_communication<HelloWorld>(
        test::dds_test_simple_configuration());
}

/**
 * Test communication in HelloWorldKeyed topic between two DDS participants created in different domains,
 * by using a router with two Simple Participants at each domain.
 */
TEST(DDSTestLocal, end_to_end_local_communication_keyed)
{
    test::test_local_communication<HelloWorldKeyed>(
        test::dds_test_simple_configuration());
}

/**
 * Test communication both in HelloWorld and HelloWorldKeyed topics between two DDS participants created in
 * different domains, by using a router with two Simple Participants at each domain.
 * In this test allowlist and blocklist are left empty, while only builtin topics are provided.
 */
TEST(DDSTestLocal, end_to_end_local_communication_disable_dynamic_discovery)
{
    test::test_local_communication<HelloWorld>(
        test::dds_test_simple_configuration(true));
}

TEST(DDSTestLocal, end_to_end_local_communication_disable_dynamic_discovery_keyed)
{
    test::test_local_communication<HelloWorldKeyed>(
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
    test::test_local_communication<HelloWorld>(
        test::dds_test_simple_configuration(),
        1000,   // wait for 1000 samples received
        0);     // send it without waiting from one sample to the other
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
    test::test_local_communication<HelloWorld>(
        test::dds_test_simple_configuration(),
        test::DEFAULT_SAMPLES_TO_RECEIVE,
        test::DEFAULT_MILLISECONDS_PUBLISH_LOOP,
        10000); // 500K message size
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
    test::test_local_communication<HelloWorld>(
        test::dds_test_simple_configuration(),
        500,
        1,
        1000); // 50K message size
}

/**
 * Test transient_local communication in HelloWorld topic between two DDS participants created in different domains,
 * by using a router with two Simple Participants at each domain.
 */
TEST(DDSTestLocal, end_to_end_local_communication_transient_local)
{
    test::test_local_communication<HelloWorld>(
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
    test::test_local_communication<HelloWorld>(
        test::dds_test_simple_configuration(true, true),
        test::DEFAULT_SAMPLES_TO_RECEIVE,
        test::DEFAULT_MILLISECONDS_PUBLISH_LOOP,
        test::DEFAULT_MESSAGE_SIZE,
        true);
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
