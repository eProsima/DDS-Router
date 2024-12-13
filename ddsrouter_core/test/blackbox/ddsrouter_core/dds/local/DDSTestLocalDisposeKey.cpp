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
#include <ddspipe_participants/configuration/SimpleParticipantConfiguration.hpp>

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

} /* namespace test */

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
