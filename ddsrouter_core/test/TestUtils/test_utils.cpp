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

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <test_utils.hpp>

namespace eprosima {
namespace ddsrouter {
namespace test {

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

RealTopicInput::RealTopicInput(
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

std::set<std::shared_ptr<RealTopic>> topic_set(
        std::vector<RealTopicInput> topics)
{
    std::set<std::shared_ptr<RealTopic>> result;
    for (RealTopicInput input : topics)
    {
        if (input.key_set)
        {
            if (input.reliable_set)
            {
                result.insert(std::make_shared<RealTopic>(
                            input.name,
                            input.type,
                            input.keyed,
                            input.reliable));
            }
            else
            {
                result.insert(std::make_shared<RealTopic>(
                            input.name,
                            input.type,
                            input.keyed));
            }
        }
        else
        {
            if (input.reliable_set)
            {
                result.insert(std::make_shared<RealTopic>(
                            input.name,
                            input.type,
                            false,
                            input.reliable));
            }
            else
            {
                result.insert(std::make_shared<RealTopic>(
                            input.name,
                            input.type));
            }
        }
    }
    return result;
}

std::set<std::shared_ptr<FilterTopic>> topic_set(
        std::vector<WildcardTopicInput> topics)
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
    return Address("127.0.0.1", seed, TransportProtocol::udp);
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

std::shared_ptr<core::configuration::ParticipantConfiguration> random_participant_configuration(
        ParticipantKind kind,
        uint16_t seed /* = 0 */)
{
    ParticipantId id("Participant" + std::to_string(seed));

    switch (kind)
    {
        case ParticipantKind::simple_rtps:
            return std::make_shared<core::configuration::SimpleParticipantConfiguration>(
                id,
                kind,
                false,
                random_domain(seed));

        case ParticipantKind::local_discovery_server:
        case ParticipantKind::wan_discovery_server:

        {
            // TODO get random values
            DiscoveryServerConnectionAddress connection_address = DiscoveryServerConnectionAddress(
                GuidPrefix(),
                std::set<Address>({Address()})
                );

            return std::make_shared<core::configuration::DiscoveryServerParticipantConfiguration>(
                id,
                kind,
                false,
                random_domain(seed),
                random_guid_prefix(seed),
                std::set<Address>(),
                std::set<DiscoveryServerConnectionAddress>({connection_address}),
                security::TlsConfiguration());
        }

        case ParticipantKind::wan_initial_peers:

        {
            return std::make_shared<core::configuration::InitialPeersParticipantConfiguration>(
                id,
                kind,
                false,
                random_domain(seed),
                std::set<Address>(),
                std::set<Address>({Address()}),
                security::TlsConfiguration());
        }

        case ParticipantKind::echo:
        {
            return std::make_shared<core::configuration::EchoParticipantConfiguration>(
                id,
                kind,
                false);
        }

        // Add cases where Participants need specific arguments
        default:
            return std::make_shared<core::configuration::ParticipantConfiguration>(id, kind, false);
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

ParticipantKind random_participant_kind(
        bool valid /* = true */,
        uint16_t seed /* = 0 */)
{
    std::vector<ParticipantKind> kinds(std::begin(ALL_VALID_PARTICIPANT_KINDS), std::end(ALL_VALID_PARTICIPANT_KINDS));
    if (valid)
    {
        return kinds[seed % kinds.size()];
    }
    else
    {
        kinds.push_back(ParticipantKind::invalid);
        return kinds[seed % kinds.size()];
    }
}

} /* namespace test */
} /* namespace ddsrouter */
} /* namespace eprosima */
