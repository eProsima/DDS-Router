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
// limitations under the License\.

#pragma once

#include <ddspipe_participants/configuration/ParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/SimpleParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/DiscoveryServerParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/EchoParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/InitialPeersParticipantConfiguration.hpp>
#include <ddspipe_core/types/dds/DomainId.hpp>
#include <ddspipe_core/types/dds/Guid.hpp>
#include <ddspipe_core/types/dds/GuidPrefix.hpp>
#include <ddspipe_core/types/topic/filter/DdsFilterTopic.hpp>
#include <ddspipe_core/types/topic/dds/DistributedTopic.hpp>
#include <ddspipe_core/types/topic/filter/WildcardDdsFilterTopic.hpp>
#include <cpp_utils/exception/InitializationException.hpp>

namespace eprosima {
namespace ddspipe {
namespace test {

using namespace eprosima::ddsrouter::core::types;

// TODO: most of the methods from this test_utils that generate random types are not very "random".
// They must be refactored to generate real random values.

constexpr const int TEST_NUMBER_ITERATIONS = 5;

/**
 * @brief Create a \c Guid with some of its bits determined by the input
 *
 * @param [in] seed : differentiating value for guid creation
 * @return generated Guid
 * @todo Make truly random using \c seed as such
 *
 */
Guid random_guid(
        uint16_t seed = 1);

struct TopicInput
{
    TopicInput(
            std::string name,
            std::string type,
            bool keyed,
            bool key_set);

    std::string name;
    std::string type;
    bool keyed;
    bool key_set;
};

struct DdsTopicInput : public TopicInput
{
    DdsTopicInput(
            std::string name,
            std::string type,
            bool keyed,
            bool key_set,
            bool reliable,
            bool reliable_set);

    bool reliable;
    bool reliable_set;
};

struct WildcardTopicInput : public TopicInput
{
    WildcardTopicInput(
            std::string name,
            std::string type,
            bool keyed,
            bool key_set,
            bool type_set);

    bool type_set;
};

std::set<std::shared_ptr<DistributedTopic>> topic_set(
        std::vector<DdsTopicInput> topics);

std::set<std::shared_ptr<DdsFilterTopic>> topic_set(
        std::vector<WildcardTopicInput> topics);

DomainId random_domain(
        uint16_t seed = 0);

GuidPrefix random_guid_prefix(
        uint16_t seed = 0,
        bool ros = false);

Address random_address(
        uint16_t seed = 0);

std::set<DiscoveryServerConnectionAddress> random_connection_addresses(
        uint16_t seed = 0,
        uint16_t size = 1,
        bool ros = false);

std::shared_ptr<core::ParticipantConfiguration> random_participant_configuration(
        ParticipantKind kind,
        uint16_t seed = 0);

ParticipantId random_participant_id(
        uint16_t seed = 0);

ParticipantKind random_participant_kind(
        bool valid = true,
        uint16_t seed = 0);

} /* namespace test */
} /* namespace ddspipe */
} /* namespace eprosima */
