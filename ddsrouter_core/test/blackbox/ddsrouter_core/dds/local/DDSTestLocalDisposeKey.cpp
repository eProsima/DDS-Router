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
#include <ddsrouter_core/types/topic/filter/DdsFilterTopic.hpp>
#include <ddsrouter_utils/Log.hpp>

#include <test_participants.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
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
 * @return configuration::DDSRouterConfiguration
 */
configuration::DDSRouterConfiguration dds_test_simple_configuration()
{
    // Always filter the test topics by topic name
    std::set<std::shared_ptr<types::DdsFilterTopic>> allowlist;   // only this topic
    allowlist.insert(std::make_shared<types::WildcardDdsFilterTopic>(TOPIC_NAME));

    std::set<std::shared_ptr<types::DdsFilterTopic>> blocklist;   // empty

    std::set<std::shared_ptr<types::DdsTopic>> builtin_topics;   // empty

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

    // TODO: this could be removed to use default number of threads
    auto specs = configuration::SpecsConfiguration();
    specs.number_of_threads = 2;

    return configuration::DDSRouterConfiguration(
        allowlist,
        blocklist,
        builtin_topics,
        participants_configurations,
        specs);
}

/**
 * Test communication between two DDS Participants hosted in the same device, but which are at different DDS domains.
 * This is accomplished by using a DDS Router instance with a Simple Participant deployed at each domain.
 *
 * The reliable option changes the test behavior to verify that the communication is reliable and all old data is sent
 * to Late Joiners.
 */
void test_local_communication_key_dispose(
        configuration::DDSRouterConfiguration ddsrouter_configuration,
        uint32_t samples_to_receive = DEFAULT_SAMPLES_TO_RECEIVE,
        uint32_t time_between_samples = DEFAULT_MILLISECONDS_PUBLISH_LOOP,
        uint32_t msg_size = DEFAULT_MESSAGE_SIZE)
{
    // Check there are no warnings/errors
    // TODO: Change threshold to \c Log::Kind::Warning once middleware warnings are solved
    eprosima::ddsrouter::test::TestLogHandler test_log_handler(utils::Log::Kind::Error);

    uint32_t samples_sent = 0;
    std::atomic<uint32_t> samples_received(0);

    // Create a message with size specified by repeating the same string
    HelloWorldKeyed msg;
    std::string msg_str;

    // Add this string as many times as the msg size requires
    for (uint32_t i = 0; i < msg_size; i++)
    {
        msg_str += "Testing DDSRouter Blackbox Local Communication ...";
    }
    msg.message(msg_str);
    msg.id(666);

    // Create DDS Publisher in domain 0
    TestPublisher<HelloWorldKeyed> publisher(msg.isKeyDefined());
    ASSERT_TRUE(publisher.init(0));

    // Create DDS Subscriber in domain 1
    TestSubscriber<HelloWorldKeyed> subscriber(msg.isKeyDefined());
    ASSERT_TRUE(subscriber.init(1, &msg, &samples_received));

    // Create DDSRouter entity
    // The DDS Router does not start here in order to test a reliable communication
    DDSRouter router(ddsrouter_configuration);

    // Start DDS Router
    router.start();

    // Start publishing
    while (samples_received.load() < samples_to_receive)
    {
        msg.index(++samples_sent);
        ASSERT_TRUE(publisher.publish(msg)) << samples_sent;

        // If time is 0 do not wait
        if (time_between_samples > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(time_between_samples));
        }
    }

    // All samples received, now dispose key from publisher and check that subscriber has receive it
    ASSERT_TRUE(publisher.dispose_key(msg) == ReturnCode_t::RETCODE_OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(DEFAULT_SUBSCRIBER_MESSAGE_RECEPTION));
    samples_received.store(0);

    // Keep publishing
    while (samples_received.load() < samples_to_receive)
    {
        msg.index(++samples_sent);
        ASSERT_TRUE(publisher.publish(msg)) << samples_sent;

        // If time is 0 do not wait
        if (time_between_samples > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(time_between_samples));
        }
    }

    // All samples received, now dispose key from publisher and check that subscriber has receive it
    ASSERT_TRUE(publisher.dispose_key(msg) == ReturnCode_t::RETCODE_OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(DEFAULT_SUBSCRIBER_MESSAGE_RECEPTION));

    ASSERT_EQ(2u, subscriber.n_disposed());

    router.stop();
}

} /* namespace test */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

using namespace eprosima::ddsrouter::core;
using namespace eprosima::ddsrouter::core::types;

/**
 * Test that dispose values from the publisher are correctly received by the subscriber from the router.
 */
TEST(DDSTestLocalDisposeKey, end_to_end_local_communication_key_dispose)
{
    test::test_local_communication_key_dispose(
        test::dds_test_simple_configuration());
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
