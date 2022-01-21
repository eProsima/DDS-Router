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

#include <ddsrouter/core/DDSRouter.hpp>
#include <ddsrouter/configuration/DDSRouterConfiguration.hpp>
#include <ddsrouter/configuration/participant/ParticipantConfiguration.hpp>
#include <ddsrouter/configuration/participant/SimpleParticipantConfiguration.hpp>
#include <ddsrouter/configuration/participant/DiscoveryServerParticipantConfiguration.hpp>
#include <ddsrouter/exceptions/InitializationException.hpp>
#include <ddsrouter/types/endpoint/DomainId.hpp>
#include <ddsrouter/types/endpoint/GuidPrefix.hpp>
#include <ddsrouter/types/topic/FilterTopic.hpp>
#include <ddsrouter/types/topic/RealTopic.hpp>
#include <ddsrouter/types/topic/WildcardTopic.hpp>
#include <ddsrouter/types/utils.hpp>
#include <ddsrouter/types/Log.hpp>

using namespace eprosima::ddsrouter;

void set_allowed_topic(
        RawConfiguration& configuration,
        std::string topic_name = "__test_topic_ddsrouter__",
        std::string topic_type = "__test_topic_type_ddsrouter__")
{
    RawConfiguration topic;
    topic[TOPIC_NAME_TAG] = topic_name;
    topic[TOPIC_TYPE_NAME_TAG] = topic_type;

    RawConfiguration allow_list;
    allow_list.push_back(topic);

    configuration[ALLOWLIST_TAG] = allow_list;
}

void set_domain(
        RawConfiguration& configuration,
        uint16_t seed = 0)
{
    configuration[DOMAIN_ID_TAG] = seed;
}

RawConfiguration participant_configuration(
        ParticipantKind kind,
        uint16_t value = 0)
{
    RawConfiguration participant_configuration;

    RawConfiguration address;
    address[ADDRESS_IP_TAG] = "127.0.0.1";
    address[ADDRESS_PORT_TAG] = 11666 + value;

    participant_configuration[PARTICIPANT_KIND_TAG] = kind.to_string();

    switch (kind())
    {
        case ParticipantKind::SIMPLE_RTPS:
            set_domain(participant_configuration);
            break;

        case ParticipantKind::LOCAL_DISCOVERY_SERVER:
            participant_configuration[LISTENING_ADDRESSES_TAG].push_back(address); // TODO: make it from method
            break;

        case ParticipantKind::WAN:
            participant_configuration[LISTENING_ADDRESSES_TAG].push_back(address); // TODO: make it from method
            break;

using namespace eprosima::ddsrouter;

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
        ASSERT_THROW(DDSRouter router(configuration), InitializationException) << kind;
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

<<<<<<< HEAD
        // Add two participants
        configuration["participant_1"] = participant_configuration(kind, 1);
        configuration["participant_2"] = participant_configuration(kind, 2);
            std::set<std::shared_ptr<RealTopic>>(),
            participant_configurations);
>>>>>>> b01029e... Refs #13530: fix tests (except implementation)

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

        uint16_t participant_number = 1;

        // For each Participant Kind set it in configuration
        for (ParticipantKind kind : ParticipantKind::all_valid_participant_kinds())
        {
            // Add participant
            participant_configurations.insert(test::random_participant_configuration(kind, ++participant_number));
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

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
