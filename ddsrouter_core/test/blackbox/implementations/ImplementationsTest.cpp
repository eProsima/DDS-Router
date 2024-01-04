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

#include <cpp_utils/exception/InitializationException.hpp>
#include <cpp_utils/exception/ConfigurationException.hpp>
#include <cpp_utils/Log.hpp>
#include <cpp_utils/utils.hpp>
#include <cpp_utils/testing/LogChecker.hpp>

#include <ddsrouter_core/core/DdsRouter.hpp>
#include <ddsrouter_core/configuration/DdsRouterConfiguration.hpp>
#include <ddsrouter_core/testing/random_values.hpp>

using namespace eprosima;
using namespace eprosima::ddsrouter::core;
using namespace eprosima::ddsrouter::core::types;
using namespace eprosima::ddsrouter::core::testing;

/**
 * Test that creates a DdsRouter with a Pair of Participants of same kind.
 * It creates a DdsRouter with two Participants of same kind, starts it, then stops it and finally destroys it.
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
    for (ParticipantKind kind : VALUES_ParticipantKind)
    {
        // Generate configuration
        DdsRouterConfiguration configuration;
        configuration.participants_configurations.insert(
        {
            kind,
            random_participant_configuration(kind, 0)
        }
            );
        configuration.participants_configurations.insert(
        {
            kind,
            random_participant_configuration(kind, 1)
        }
            );

        // Create DdsRouter entity
        DdsRouter router(configuration);

        // Start DdsRouter
        router.start();

        // Stop DDS Router
        router.stop();

        // Let DdsRouter object destroy for the next iteration
    }
}

/**
 * Test that creates a DdsRouter with a Pair of Participants of same kind.
 * It creates a DdsRouter with two Participants of same kind, starts it with an active topic,
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
    for (ParticipantKind kind : VALUES_ParticipantKind)
    {
        // Generate configuration
        DdsRouterConfiguration configuration;
        configuration.participants_configurations.insert(
        {
            kind,
            random_participant_configuration(kind, 0)
        }
            );
        configuration.participants_configurations.insert(
        {
            kind,
            random_participant_configuration(kind, 1)
        }
            );

        // Add topic
        eprosima::ddspipe::core::types::DdsTopic topic;
        topic.m_topic_name = "rt/chatter";
        topic.type_name = "std_msgs::msg::dds_::String_";
        configuration.ddspipe_configuration.builtin_topics.insert(
            utils::Heritable<eprosima::ddspipe::core::types::DdsTopic>::make_heritable(topic));

        // Create DdsRouter entity
        DdsRouter router(configuration);

        // Start DdsRouter
        router.start();

        // Stop DDS Router
        router.stop();

        // Let DdsRouter object destroy for the next iteration
    }
}

/**
 * Test that creates a DdsRouter with several Participants, one of each kind
 * It creates a DdsRouter with a Participant of each kind,
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
        DdsRouterConfiguration configuration;

        // For each Participant Kind set it in configuration
        uint16_t participant_number = 0;
        for (ParticipantKind kind : VALUES_ParticipantKind)
        {
            // Add participant
            configuration.participants_configurations.insert(
            {
                kind,
                random_participant_configuration(kind, participant_number++)
            }
                );
        }

        // Create DdsRouter entity
        DdsRouter router(configuration);

        // Start DdsRouter
        router.start();

        // Stop DDS Router
        router.stop();

        // Let DdsRouter object destroy for the next iteration
    }
}

/**
 * Test that creates a DdsRouter with 3 simple configurations, 2 of them with same id, fails
 *
 * There is no easy way to test this case as the yaml will be ill-formed with two keys.
 * Thus, it must be implemented from a yaml in string format.
 */
TEST(ImplementationsTest, duplicated_ids_negative)
{
    DdsRouterConfiguration configuration;
    configuration.participants_configurations.insert(
    {
        ParticipantKind::simple,
        random_participant_configuration(ParticipantKind::simple, 0)
    }
        );
    configuration.participants_configurations.insert(
    {
        ParticipantKind::simple,
        random_participant_configuration(ParticipantKind::simple, 0)
    }
        );

    // Create DdsRouter entity
    ASSERT_THROW(DdsRouter router(configuration), eprosima::utils::ConfigurationException);
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
