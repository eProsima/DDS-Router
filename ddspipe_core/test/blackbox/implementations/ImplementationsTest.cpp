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

#include <algorithm>
#include <iostream>

#include <cpp_utils/testing/gtest_aux.hpp>
#include <gtest/gtest.h>
#include <test_utils.hpp>

#include <cpp_utils/testing/LogChecker.hpp>

#include <ddspipe_core/core/DDSRouter.hpp>
#include <cpp_utils/exception/ConfigurationException.hpp>
#include <ddspipe_core/configuration/DDSRouterConfiguration.hpp>
#include <ddspipe_core/participants/participant/configuration/ParticipantConfiguration.hpp>
#include <ddspipe_core/participants/participant/configuration/SimpleParticipantConfiguration.hpp>
#include <ddspipe_core/participants/participant/configuration/DiscoveryServerParticipantConfiguration.hpp>
#include <cpp_utils/exception/InitializationException.hpp>
#include <ddspipe_core/types/dds/DomainId.hpp>
#include <ddspipe_core/types/dds/GuidPrefix.hpp>
#include <ddspipe_core/types/topic/filter/DdsFilterTopic.hpp>
#include <ddspipe_core/types/topic/dds/DistributedTopic.hpp>
#include <ddspipe_core/types/topic/filter/WildcardDdsFilterTopic.hpp>
#include <cpp_utils/utils.hpp>
#include <cpp_utils/Log.hpp>

namespace eprosima {
namespace ddspipe {
namespace test {

constexpr const unsigned int DEFAULT_THREAD_POOL_SIZE = 2;
constexpr const unsigned int DEFAULT_MAX_HISTORY_DEPTH = 100;

} /* namespace test */
} /* namespace ddspipe */
} /* namespace eprosima */

using namespace eprosima::ddsrouter;
using namespace eprosima::ddsrouter::core;
using namespace eprosima::ddsrouter::core::types;

/**
 * Test that creates a DDSRouter with a Pair of Participants of same kind.
 * It creates a DDSRouter with two Participants of same kind, starts it, then stops it and finally destroys it.
 *
 * This test will fail if it crashes.
 */
TEST(ImplementationsTest, pair_implementation)
{
    // TODO: Change to warning when fastdds warning:
    // [RTPS_MSG_OUT Warning] Error receiving data: receive_from: A blocking operation was interrupted by a call to WSACancelBlockingCall.:
    // A blocking operation was interrupted by a call to WSACancelBlockingCall. - 0000016CEBD18C10 (0000016CEBD17A40) ->
    // Function eprosima::fastdds::rtps::UDPChannelResource::Receive
    // test::LogChecker test_log_handler(utils::Log::Kind::Error);
    INSTANTIATE_LOG_TESTER(eprosima::utils::Log::Kind::Error, 0, 0);

    // For each Participant Kind
    for (ParticipantKind kind : ALL_VALID_PARTICIPANT_KINDS)
    {
        std::set<std::shared_ptr<configuration::ParticipantConfiguration>> participant_configurations;
        participant_configurations.insert(test::random_participant_configuration(kind, 1));
        participant_configurations.insert(test::random_participant_configuration(kind, 2));

        configuration::SpecsConfiguration specs;
        specs.max_history_depth = test::DEFAULT_MAX_HISTORY_DEPTH;
        specs.number_of_threads = test::DEFAULT_THREAD_POOL_SIZE;

        // Generate configuration
        configuration::DDSRouterConfiguration configuration(
            std::set<std::shared_ptr<DdsFilterTopic>>(),
            std::set<std::shared_ptr<DdsFilterTopic>>(),
            std::set<std::shared_ptr<DistributedTopic>>(),
            participant_configurations,
            specs);

        // Create DDSRouter entity
        DDSRouter router(configuration);
        // Start DDSRouter
        router.start();

        // Stop DDS Router
        router.stop();

        // Let DDSRouter object destroy for the next iteration
    }
}

/**
 * Test that creates a DDSRouter with a Pair of Participants of same kind.
 * It creates a DDSRouter with two Participants of same kind, starts it with an active topic,
 * then stops it and finally destroys it.
 *
 * This test will fail if it crashes.
 */
TEST(ImplementationsTest, pair_implementation_with_topic)
{
    // TODO: Change to warning when fastdds warning:
    // [RTPS_MSG_OUT Warning] Error receiving data: receive_from: A blocking operation was interrupted by a call to WSACancelBlockingCall.:
    // A blocking operation was interrupted by a call to WSACancelBlockingCall. - 0000016CEBD18C10 (0000016CEBD17A40) ->
    // Function eprosima::fastdds::rtps::UDPChannelResource::Receive
    // test::LogChecker test_log_handler(utils::Log::Kind::Error);
    INSTANTIATE_LOG_TESTER(eprosima::utils::Log::Kind::Error, 0, 0);

    // For each Participant kind
    for (ParticipantKind kind : ALL_VALID_PARTICIPANT_KINDS)
    {
        std::set<std::shared_ptr<DistributedTopic>> builtin_topics = test::topic_set(
            {test::DdsTopicInput("rt/chatter", "std_msgs::msg::dds_::String_", false, false, false, false)});

        std::set<std::shared_ptr<configuration::ParticipantConfiguration>> participant_configurations;
        participant_configurations.insert(test::random_participant_configuration(kind, 1));
        participant_configurations.insert(test::random_participant_configuration(kind, 2));

        configuration::SpecsConfiguration specs;
        specs.max_history_depth = test::DEFAULT_MAX_HISTORY_DEPTH;
        specs.number_of_threads = test::DEFAULT_THREAD_POOL_SIZE;

        // Generate configuration
        configuration::DDSRouterConfiguration configuration(
            std::set<std::shared_ptr<DdsFilterTopic>>(),
            std::set<std::shared_ptr<DdsFilterTopic>>(),
            builtin_topics,
            participant_configurations,
            specs);

        // Create DDSRouter entity
        DDSRouter router(configuration);

        // Start DDSRouter
        router.start();

        // Stop DDS Router
        router.stop();

        // Let DDSRouter object destroy for the next iteration
    }
}

/**
 * Test that creates a DDSRouter with several Participants, one of each kind
 * It creates a DDSRouter with a Participant of each kind,
 * starts it with an active topic, then stops it and finally destroys it.
 *
 * This test will fail if it crashes.
 */
TEST(ImplementationsTest, all_implementations)
{
    // TODO: Change to warning when fastdds warning:
    // [RTPS_MSG_OUT Warning] Error receiving data: receive_from: A blocking operation was interrupted by a call to WSACancelBlockingCall.:
    // A blocking operation was interrupted by a call to WSACancelBlockingCall. - 0000016CEBD18C10 (0000016CEBD17A40) ->
    // Function eprosima::fastdds::rtps::UDPChannelResource::Receive
    // test::LogChecker test_log_handler(utils::Log::Kind::Error);
    INSTANTIATE_LOG_TESTER(eprosima::utils::Log::Kind::Error, 0, 0);

    {
        // Set topic to active
        std::set<std::shared_ptr<DistributedTopic>> builtin_topics = test::topic_set(
            {test::DdsTopicInput("rt/chatter", "std_msgs::msg::dds_::String_", false, false, false, false)});

        std::set<std::shared_ptr<configuration::ParticipantConfiguration>> participant_configurations;

        uint16_t participant_number = 0;

        // For each Participant Kind set it in configuration
        for (ParticipantKind kind : ALL_VALID_PARTICIPANT_KINDS)
        {
            // Add participant
            participant_configurations.insert(test::random_participant_configuration(kind, participant_number++));
        }

        configuration::SpecsConfiguration specs;
        specs.max_history_depth = test::DEFAULT_MAX_HISTORY_DEPTH;
        specs.number_of_threads = test::DEFAULT_THREAD_POOL_SIZE;

        // Generate configuration
        configuration::DDSRouterConfiguration configuration(
            std::set<std::shared_ptr<DdsFilterTopic>>(),
            std::set<std::shared_ptr<DdsFilterTopic>>(),
            std::set<std::shared_ptr<DistributedTopic>>(),
            participant_configurations,
            specs);

        // Create DDSRouter entity
        DDSRouter router(configuration);

        // Start DDSRouter
        router.start();

        // Stop DDS Router
        router.stop();

        // Let DDSRouter object destroy for the next iteration
    }
}

/**
 * Test that creates a DDSRouter with 3 simple configurations, 2 of them with same id, fails
 *
 * There is no easy way to test this case as the yaml will be ill-formed with two keys.
 * Thus, it must be implemented from a yaml in string format.
 */
TEST(ImplementationsTest, duplicated_ids)
{
    // For each Participant Kind
    for (ParticipantKind kind : ALL_VALID_PARTICIPANT_KINDS)
    {
        std::set<std::shared_ptr<configuration::ParticipantConfiguration>> participant_configurations;
        participant_configurations.insert(test::random_participant_configuration(kind, 0));
        participant_configurations.insert(test::random_participant_configuration(kind, 0));

        configuration::SpecsConfiguration specs;
        specs.max_history_depth = test::DEFAULT_MAX_HISTORY_DEPTH;
        specs.number_of_threads = test::DEFAULT_THREAD_POOL_SIZE;

        // Generate configuration
        configuration::DDSRouterConfiguration configuration(
            std::set<std::shared_ptr<DdsFilterTopic>>(),
            std::set<std::shared_ptr<DdsFilterTopic>>(),
            std::set<std::shared_ptr<DistributedTopic>>(),
            participant_configurations,
            specs);

        // Create DDSRouter entity
        ASSERT_THROW(DDSRouter router(configuration), eprosima::utils::ConfigurationException) << kind;
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
