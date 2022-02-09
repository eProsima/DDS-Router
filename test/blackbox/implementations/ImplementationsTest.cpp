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
#include <TestLogHandler.hpp>

#include <ddsrouter/core/DDSRouter.hpp>
#include <ddsrouter/exceptions/ConfigurationException.hpp>
#include <ddsrouter/exceptions/InitializationException.hpp>
#include <ddsrouter/participant/implementations/auxiliar/DummyParticipant.hpp>
#include <ddsrouter/types/configuration_tags.hpp>
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
        ParticipantType type,
        uint16_t value = 0)
{
    RawConfiguration participant_configuration;

    RawConfiguration address;
    address[ADDRESS_IP_TAG] = "127.0.0.1";
    address[ADDRESS_PORT_TAG] = 11666 + value;

    participant_configuration[PARTICIPANT_TYPE_TAG] = type.to_string();

    switch (type())
    {
        case ParticipantType::SIMPLE_RTPS:
            set_domain(participant_configuration);
            break;

        case ParticipantType::LOCAL_DISCOVERY_SERVER:
            participant_configuration[LISTENING_ADDRESSES_TAG].push_back(address); // TODO: make it from method
            break;

        case ParticipantType::WAN:
            participant_configuration[LISTENING_ADDRESSES_TAG].push_back(address); // TODO: make it from method
            break;

        // Add cases where Participants need specific arguments
        default:
            break;
    }

    static_cast<void>(value);

    return participant_configuration;
}

/**
 * Test that tries to create a DDSRouter with only one Participant.
 *
 * It expects to receive an exception
 */
TEST(ImplementationsTest, solo_participant_implementation)
{
    // For each Participant Type
    for (ParticipantType type : ParticipantType::all_valid_participant_types())
    {
        // Generate configuration
        RawConfiguration configuration;

        // Add two participants
        configuration["participant_1"] = participant_configuration(1);

        // Create DDSRouter entity
        ASSERT_THROW(DDSRouter router(configuration), InitializationException);
    }
}

/**
 * Test that creates a DDSRouter with a Pair of Participants of same type.
 * It creates a DDSRouter with two Participants of same kind, starts it, then stops it and finally destroys it.
 *
 * This test will fail if it crashes.
 */
TEST(ImplementationsTest, pair_implementation)
{
    test::TestLogHandler test_log_handler;

    // For each Participant Type
    for (ParticipantType type : ParticipantType::all_valid_participant_types())
    {
        // Generate configuration
        RawConfiguration configuration;

        // Add two participants
        configuration["participant_1"] = participant_configuration(1);
        configuration["participant_2"] = participant_configuration(2);

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
 * Test that creates a DDSRouter with a Pair of Participants of same type.
 * It creates a DDSRouter with two Participants of same kind, starts it with an active topic,
 * then stops it and finally destroys it.
 *
 * This test will fail if it crashes.
 */
TEST(ImplementationsTest, pair_implementation_with_topic)
{
    test::TestLogHandler test_log_handler;

    // For each Participant Type
    for (ParticipantType type : ParticipantType::all_valid_participant_types())
    {
        // Generate configuration
        RawConfiguration configuration;

        // Add two participants
        configuration["participant_1"] = participant_configuration(1);
        configuration["participant_2"] = participant_configuration(2);

        // Set topic to active
        set_allowed_topic(configuration);

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
 * Test that creates a DDSRouter with several Participants, one of each type
 * It creates a DDSRouter with a Participant of each kind,
 * starts it with an active topic, then stops it and finally destroys it.
 *
 * This test will fail if it crashes.
 */
TEST(ImplementationsTest, all_implementations)
{
    test::TestLogHandler test_log_handler;

    {
        // Generate configuration
        RawConfiguration configuration;

        uint16_t participant_number = 1;

        // For each Participant Type set it in configuration
        for (ParticipantType type : ParticipantType::all_valid_participant_types())
        {
            // Add participant
            std::string participant_name = "participant_" + type.to_string();
            configuration[participant_name] = participant_configuration(type, ++participant_number);
        }

        // Set topic to active
        set_allowed_topic(configuration);

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
    const char *yaml_str = R"(
        participant_1:
            type: simple
            domain: 0

        participant_2:
            type: simple
            domain: 1

        participant_2:
            type: simple
            domain: 2
        )";

    RawConfiguration configuration(yaml_str);

    ASSERT_THROW(DDSRouter ddsrouter(configuration), ConfigurationException);
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
