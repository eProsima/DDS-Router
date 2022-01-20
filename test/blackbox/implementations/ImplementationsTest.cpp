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
#include <ddsrouter/configuration/DDSRouterConfiguration.hpp>
#include <ddsrouter/configuration/participant/ParticipantConfiguration.hpp>
#include <ddsrouter/configuration/participant/SimpleParticipantConfiguration.hpp>
#include <ddsrouter/configuration/participant/DiscoveryServerParticipantConfiguration.hpp>
#include <ddsrouter/exception/InitializationException.hpp>
#include <ddsrouter/types/endpoint/DomainId.hpp>
#include <ddsrouter/types/endpoint/GuidPrefix.hpp>
#include <ddsrouter/types/topic/FilterTopic.hpp>
#include <ddsrouter/types/topic/RealTopic.hpp>
#include <ddsrouter/types/topic/WildcardTopic.hpp>
#include <ddsrouter/types/utils.hpp>
#include <ddsrouter/types/Log.hpp>

using namespace eprosima::ddsrouter;

struct TopicInput
{
    std::string name;
    std::string type;
    bool keyed;
    bool key_set;
};

struct WildcardTopicInput : public TopicInput
{
    bool type_set;
};

std::set<std::shared_ptr<RealTopic>> topic_set(std::vector<TopicInput> topics)
{
    std::set<std::shared_ptr<RealTopic>> result;
    for (TopicInput input : topics)
    {
        if (input.key_set)
        {
            result.insert(std::make_shared<RealTopic>(
                input.name,
                input.type,
                input.keyed));
        }
        else
        {
            result.insert(std::make_shared<RealTopic>(
                input.name,
                input.type));
        }
    }
    return result;
}

std::set<std::shared_ptr<FilterTopic>> topic_set(std::vector<WildcardTopicInput> topics)
{
    std::set<std::shared_ptr<FilterTopic>> result;
    for (WildcardTopicInput input : topics)
    {
        if (input.key_set)
        {
            if (input.type_set)
            {
                result.insert(std::make_shared<WildcardTopic>(
                    input.name,
                    input.type,
                    true,
                    input.keyed));
            }
            else
            {
                result.insert(std::make_shared<WildcardTopic>(
                    input.name,
                    true,
                    input.keyed));
            }
        }
        else
        {
            if (input.type_set)
            {
                result.insert(std::make_shared<WildcardTopic>(
                    input.name,
                    input.type,
                    false));
            }
            else
            {
                result.insert(std::make_shared<WildcardTopic>(
                    input.name,
                    false));
            }
        }
    }
    return result;
}

DomainId random_domain(
        uint16_t seed = 0)
{
    return DomainId(static_cast<DomainIdType>(seed));
}

GuidPrefix random_guid_prefix(
        uint16_t seed = 0,
        bool ros = false)
{
    if (ros)
    {
        return GuidPrefix(true, seed);
    }
    else
    {
        return GuidPrefix(static_cast<uint32_t>(seed));
    }
}

Address random_address(
        uint16_t seed = 0)
{
    return Address();
}

std::set<DiscoveryServerConnectionAddress> random_connection_addresses(
        uint16_t seed = 0,
        uint16_t size = 1,
        bool ros = false)
{
    std::set<DiscoveryServerConnectionAddress> result;

    for (int i=0; i<size; ++i)
    {
        result.insert(
            DiscoveryServerConnectionAddress(
                random_guid_prefix((seed * size + i) * i),
                std::set<Address>({
                    random_address((seed * size + i) * i),
                    random_address((seed * size + i) * i + 1)})));
    }
    return result;
}

std::shared_ptr<configuration::ParticipantConfiguration> random_participant_configuration(
        ParticipantKind kind,
        uint16_t seed = 0)
{
    ParticipantId id("Participant" + std::to_string(seed));

    switch (kind())
    {
        case ParticipantKind::SIMPLE_RTPS:
            return std::make_shared<configuration::SimpleParticipantConfiguration>(
                id,
                kind,
                random_domain(seed));

        case ParticipantKind::LOCAL_DISCOVERY_SERVER:
        case ParticipantKind::WAN:
            return std::make_shared<configuration::DiscoveryServerParticipantConfiguration>(
                id,
                random_guid_prefix(seed),
                std::set<Address>(),
                std::set<DiscoveryServerConnectionAddress>(),
                kind);

        // Add cases where Participants need specific arguments
        default:
            return std::make_shared<configuration::ParticipantConfiguration>(id, kind);
    }
}

/**
 * Test that tries to create a DDSRouter with only one Participant.
 *
 * It expects to receive an exception
 */
TEST(ImplementationsTest, solo_participant_implementation)
{
    // For each Participant Type
    for (ParticipantKind kind : ParticipantKind::all_valid_participant_kinds())
    {
        std::set<std::shared_ptr<configuration::ParticipantConfiguration>> participant_configurations;
        participant_configurations.insert(random_participant_configuration(kind));

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
 * Test that creates a DDSRouter with a Pair of Participants of same type.
 * It creates a DDSRouter with two Participants of same kind, starts it, then stops it and finally destroys it.
 *
 * This test will fail if it crashes.
 */
TEST(ImplementationsTest, pair_implementation)
{
    test::TestLogHandler test_log_handler;

    // For each Participant Type
    for (ParticipantKind kind : ParticipantKind::all_valid_participant_kinds())
    {
        std::set<std::shared_ptr<configuration::ParticipantConfiguration>> participant_configurations;
        participant_configurations.insert(random_participant_configuration(kind, 1));
        participant_configurations.insert(random_participant_configuration(kind, 2));

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
    for (ParticipantKind kind : ParticipantKind::all_valid_participant_kinds())
    {
        std::set<std::shared_ptr<RealTopic>> builtin_topics = topic_set(
            {{"rt/chatter", "std_msgs::msg::dds_::String_", false, false}});

        std::set<std::shared_ptr<configuration::ParticipantConfiguration>> participant_configurations;
        participant_configurations.insert(random_participant_configuration(kind, 1));
        participant_configurations.insert(random_participant_configuration(kind, 2));

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
        // Set topic to active
        std::set<std::shared_ptr<RealTopic>> builtin_topics = topic_set(
            {{"rt/chatter", "std_msgs::msg::dds_::String_", false, false}});

        std::set<std::shared_ptr<configuration::ParticipantConfiguration>> participant_configurations;

        uint16_t participant_number = 1;

        // For each Participant Type set it in configuration
        for (ParticipantKind kind : ParticipantKind::all_valid_participant_kinds())
        {
            // Add participant
            participant_configurations.insert(random_participant_configuration(kind, ++participant_number));
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
