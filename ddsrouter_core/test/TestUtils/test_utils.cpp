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

std::set<std::shared_ptr<RealTopic>> topic_set(
        std::vector<TopicInput> topics)
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
    return Address("127.0.0.1", seed, UDP);
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

    switch (kind())
    {
        case ParticipantKind::SIMPLE_RTPS:
            return std::make_shared<core::configuration::SimpleParticipantConfiguration>(
                id,
                kind,
                random_domain(seed));

        case ParticipantKind::LOCAL_DISCOVERY_SERVER:
        case ParticipantKind::WAN:

        {
            // TODO get random values
            DiscoveryServerConnectionAddress connection_address = DiscoveryServerConnectionAddress(
                GuidPrefix(),
                std::set<Address>({Address()})
                );

            return std::make_shared<core::configuration::DiscoveryServerParticipantConfiguration>(
                id,
                random_guid_prefix(seed),
                std::set<Address>(),
                std::set<DiscoveryServerConnectionAddress>({connection_address}),
                kind);
        }

        // Add cases where Participants need specific arguments
        default:
            return std::make_shared<core::configuration::ParticipantConfiguration>(id, kind);
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
    std::vector<ParticipantKind> kinds = ParticipantKind::all_valid_participant_kinds();
    if (valid)
    {
        return kinds[seed % kinds.size()];
    }
    else
    {
        kinds.push_back(ParticipantKind::PARTICIPANT_KIND_INVALID);
        return kinds[seed % kinds.size()];
    }
}

} /* namespace test */
} /* namespace ddsrouter */
} /* namespace eprosima */
