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

#include <gtest_aux.hpp>
#include <gtest/gtest.h>
#include <test_utils.hpp>
#include <TestLogHandler.hpp>

#include <ddsrouter_core/core/DDSRouter.hpp>
#include <ddsrouter_utils/exception/ConfigurationException.hpp>
#include <ddsrouter_core/configuration/DDSRouterConfiguration.hpp>
#include <ddsrouter_core/configuration/participant/ParticipantConfiguration.hpp>
#include <ddsrouter_core/configuration/participant/SimpleParticipantConfiguration.hpp>
#include <ddsrouter_core/configuration/participant/DiscoveryServerParticipantConfiguration.hpp>
#include <ddsrouter_utils/exception/InitializationException.hpp>
#include <ddsrouter_core/types/dds/DomainId.hpp>
#include <ddsrouter_core/types/dds/GuidPrefix.hpp>
#include <ddsrouter_core/types/topic/FilterTopic.hpp>
#include <ddsrouter_core/types/topic/RealTopic.hpp>
#include <ddsrouter_core/types/topic/WildcardTopic.hpp>
#include <ddsrouter_utils/utils.hpp>
#include <ddsrouter_utils/Log.hpp>

using namespace eprosima::ddsrouter;
using namespace eprosima::ddsrouter::core;
using namespace eprosima::ddsrouter::core::types;

/**
 * Test that tries to create a DDSRouter with only one Participant.
 *
 * It expects to receive an exception
 */
TEST(ImplementationsTest, solo_participant_implementation)
{
    // For each Participant Kind
    for (ParticipantKind kind : ParticipantKind::all_valid_participant_kinds())
    {
        std::set<std::shared_ptr<configuration::ParticipantConfiguration>> participant_configurations;
        participant_configurations.insert(test::random_participant_configuration(kind));

        // Generate configuration
        configuration::DDSRouterConfiguration configuration(
            std::set<std::shared_ptr<FilterTopic>>(),
            std::set<std::shared_ptr<FilterTopic>>(),
            std::set<std::shared_ptr<RealTopic>>(),
            participant_configurations);

        // Create DDSRouter entity
        ASSERT_THROW(DDSRouter router(configuration), utils::ConfigurationException) << kind;
    }
}

/**
 * Test that creates a DDSRouter with a Pair of Participants of same kind.
 * It creates a DDSRouter with two Participants of same kind, starts it, then stops it and finally destroys it.
 *
 * This test will fail if it crashes.
 */
TEST(ImplementationsTest, pair_implementation)
{
    test::TestLogHandler test_log_handler;

    // For each Participant Kind
    for (ParticipantKind kind : ParticipantKind::all_valid_participant_kinds())
    {
        std::set<std::shared_ptr<configuration::ParticipantConfiguration>> participant_configurations;
        participant_configurations.insert(test::random_participant_configuration(kind, 1));
        participant_configurations.insert(test::random_participant_configuration(kind, 2));

        // Generate configuration
        configuration::DDSRouterConfiguration configuration(
            std::set<std::shared_ptr<FilterTopic>>(),
            std::set<std::shared_ptr<FilterTopic>>(),
            std::set<std::shared_ptr<RealTopic>>(),
            participant_configurations);

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
    test::TestLogHandler test_log_handler;

    // For each Participant kind
    for (ParticipantKind kind : ParticipantKind::all_valid_participant_kinds())
    {
        std::set<std::shared_ptr<RealTopic>> builtin_topics = test::topic_set(
            {{"rt/chatter", "std_msgs::msg::dds_::String_", false, false}});

        std::set<std::shared_ptr<configuration::ParticipantConfiguration>> participant_configurations;
        participant_configurations.insert(test::random_participant_configuration(kind, 1));
        participant_configurations.insert(test::random_participant_configuration(kind, 2));

        // Generate configuration
        configuration::DDSRouterConfiguration configuration(
            std::set<std::shared_ptr<FilterTopic>>(),
            std::set<std::shared_ptr<FilterTopic>>(),
            builtin_topics,
            participant_configurations);

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
    test::TestLogHandler test_log_handler;

    {
        // Set topic to active
        std::set<std::shared_ptr<RealTopic>> builtin_topics = test::topic_set(
            {{"rt/chatter", "std_msgs::msg::dds_::String_", false, false}});

        std::set<std::shared_ptr<configuration::ParticipantConfiguration>> participant_configurations;

        uint16_t participant_number = 0;

        // For each Participant Kind set it in configuration
        for (ParticipantKind kind : ParticipantKind::all_valid_participant_kinds())
        {
            // Add participant
            participant_configurations.insert(test::random_participant_configuration(kind, participant_number++));
        }

        // Generate configuration
        configuration::DDSRouterConfiguration configuration(
            std::set<std::shared_ptr<FilterTopic>>(),
            std::set<std::shared_ptr<FilterTopic>>(),
            std::set<std::shared_ptr<RealTopic>>(),
            participant_configurations);

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
    for (ParticipantKind kind : ParticipantKind::all_valid_participant_kinds())
    {
        std::set<std::shared_ptr<configuration::ParticipantConfiguration>> participant_configurations;
        participant_configurations.insert(test::random_participant_configuration(kind, 0));
        participant_configurations.insert(test::random_participant_configuration(kind, 0));

        // Generate configuration
        configuration::DDSRouterConfiguration configuration(
            std::set<std::shared_ptr<FilterTopic>>(),
            std::set<std::shared_ptr<FilterTopic>>(),
            std::set<std::shared_ptr<RealTopic>>(),
            participant_configurations);

        // Create DDSRouter entity
        ASSERT_THROW(DDSRouter router(configuration), eprosima::ddsrouter::utils::ConfigurationException) << kind;
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
