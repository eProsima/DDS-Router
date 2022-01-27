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
 * @file test_utils.hpp
 */

#ifndef _DDSROUTER_TEST_TESTUTILS_TEST_UTILS_HPP_
#define _DDSROUTER_TEST_TESTUTILS_TEST_UTILS_HPP_

#include <ddsrouter/types/endpoint/Guid.hpp>
#include <ddsrouter/configuration/participant/ParticipantConfiguration.hpp>
#include <ddsrouter/configuration/participant/SimpleParticipantConfiguration.hpp>
#include <ddsrouter/configuration/participant/DiscoveryServerParticipantConfiguration.hpp>
#include <ddsrouter/exceptions/InitializationException.hpp>
#include <ddsrouter/types/endpoint/DomainId.hpp>
#include <ddsrouter/types/endpoint/GuidPrefix.hpp>
#include <ddsrouter/types/topic/FilterTopic.hpp>
#include <ddsrouter/types/topic/RealTopic.hpp>
#include <ddsrouter/types/topic/WildcardTopic.hpp>

namespace eprosima {
namespace ddsrouter {
namespace test {

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
    std::string name;
    std::string type;
    bool keyed;
    bool key_set;
};

struct WildcardTopicInput : public TopicInput
{
    bool type_set;
};

std::set<std::shared_ptr<RealTopic>> topic_set(
        std::vector<TopicInput> topics);

std::set<std::shared_ptr<FilterTopic>> topic_set(
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

std::shared_ptr<configuration::ParticipantConfiguration> random_participant_configuration(
        ParticipantKind kind,
        uint16_t seed = 0);

} /* namespace test */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TEST_TESTUTILS_TEST_UTILS_HPP_ */
