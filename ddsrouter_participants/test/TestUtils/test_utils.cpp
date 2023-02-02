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

/**
 * @file test_utils.cpp
 *
 */

#include <cpp_utils/testing/gtest_aux.hpp>
#include <gtest/gtest.h>

#include <test_utils.hpp>

namespace test {

using namespace eprosima;
using namespace eprosima::ddsrouter::core::types;

Guid random_guid(
        uint16_t seed /* = 1 */)
{
    Guid guid;
    guid.entityId.value[3] = static_cast<eprosima::fastrtps::rtps::octet>(seed);
    guid.guidPrefix.value[0] = 0x01;
    guid.guidPrefix.value[1] = 0x0f;
    return guid;
}

TopicInput::TopicInput(
        std::string name,
        std::string type,
        bool keyed,
        bool key_set)
    : name(name)
    , type(type)
    , keyed(keyed)
    , key_set(key_set)
{
}

DdsTopicInput::DdsTopicInput(
        std::string name,
        std::string type,
        bool keyed,
        bool key_set,
        bool reliable,
        bool reliable_set)
    : TopicInput(name, type, keyed, key_set)
    , reliable(reliable)
    , reliable_set(reliable_set)
{
}

WildcardTopicInput::WildcardTopicInput(
        std::string name,
        std::string type,
        bool keyed,
        bool key_set,
        bool type_set)
    : TopicInput(name, type, keyed, key_set)
    , type_set(type_set)
{
}

std::set<std::shared_ptr<DdsTopic>> topic_set(
        std::vector<DdsTopicInput> topics)
{
    std::set<std::shared_ptr<DdsTopic>> result;
    for (DdsTopicInput input : topics)
    {
        auto new_topic = std::make_shared<DdsTopic>();
        new_topic->topic_name = input.name;
        new_topic->type_name = input.type;

        if (input.key_set)
        {
            new_topic->keyed = input.keyed;
        }


        if (input.reliable_set)
        {
            TopicQoS qos;
            if (input.reliable)
            {
                qos.reliability_qos = ReliabilityKind::RELIABLE;
            }
            else
            {
                qos.reliability_qos = ReliabilityKind::BEST_EFFORT;
            }
            new_topic->topic_qos = qos;
            new_topic->topic_qos.set_level(utils::FuzzyLevelValues::fuzzy_level_fuzzy);
        }

        result.insert(new_topic);
    }
    return result;
}

std::set<std::shared_ptr<DdsFilterTopic>> topic_set(
        std::vector<WildcardTopicInput> topics)
{
    std::set<std::shared_ptr<DdsFilterTopic>> result;
    for (WildcardTopicInput input : topics)
    {
        auto new_topic = std::make_shared<WildcardDdsFilterTopic>();
        new_topic->topic_name = input.name;
        new_topic->type_name = input.type;

        if (input.key_set)
        {
            new_topic->keyed = input.type_set;
        }

        result.insert(new_topic);

    }
    return result;
}

DomainId random_domain(
        uint16_t seed /* = 0 */)
{
    return DomainId(static_cast<DomainIdType>(seed));
}

GuidPrefix random_guid_prefix(
        uint16_t seed /* = 0 */,
        bool ros /* = false */)
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
        uint16_t seed /* = 0 */)
{
    return Address("127.0.0.1", seed, seed, TransportProtocol::udp);
}

std::set<DiscoveryServerConnectionAddress> random_connection_addresses(
        uint16_t seed /* = 0 */,
        uint16_t size /* = 1 */,
        bool ros /* = false */)
{
    std::set<DiscoveryServerConnectionAddress> result;

    for (int i = 0; i < size; ++i)
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

std::shared_ptr<participants::ParticipantConfiguration> random_participant_configuration(
        participants::ParticipantKind kind,
        uint16_t seed /* = 0 */)
{
    ParticipantId id("Participant" + std::to_string(seed));

    switch (kind)
    {
        case participants::ParticipantKind::simple:
            return std::make_shared<participants::SimpleParticipantConfiguration>(
                id,
                false,
                random_domain(seed));

        case participants::ParticipantKind::discovery_server:

        {
            // TODO get random values
            DiscoveryServerConnectionAddress connection_address = DiscoveryServerConnectionAddress(
                GuidPrefix(),
                std::set<Address>({Address()})
                );

            return std::make_shared<participants::DiscoveryServerParticipantConfiguration>(
                id,
                false,
                random_domain(seed),
                random_guid_prefix(seed),
                std::set<Address>(),
                std::set<DiscoveryServerConnectionAddress>({connection_address}),
                security::TlsConfiguration());
        }

        case participants::ParticipantKind::initial_peers:

        {
            return std::make_shared<participants::InitialPeersParticipantConfiguration>(
                id,
                false,
                random_domain(seed),
                std::set<Address>(),
                std::set<Address>({Address()}),
                security::TlsConfiguration());
        }

        case participants::ParticipantKind::echo:
        {
            return std::make_shared<participants::EchoParticipantConfiguration>(
                id,
                false);
        }

        // Add cases where Participants need specific arguments
        default:
            return std::make_shared<participants::ParticipantConfiguration>(id, false);
    }
}

ParticipantId random_participant_id(
        uint16_t seed /* = 0 */)
{
    std::vector<std::string> names = {
        "participant",
        "PART_1",
        "echo",
        "Barro_p",
    };

    return ParticipantId(names[seed % names.size()] + std::to_string(seed));
}

} /* namespace test */
