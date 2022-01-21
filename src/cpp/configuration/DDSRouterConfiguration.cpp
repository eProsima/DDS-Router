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
 * @file DDSRouterConfiguration.cpp
 *
 */

#include <ddsrouter/configuration/DDSRouterConfiguration.hpp>
#include <ddsrouter/types/Log.hpp>
#include <ddsrouter/types/topic/WildcardTopic.hpp>
#include <ddsrouter/exceptions/ConfigurationException.hpp>

namespace eprosima {
namespace ddsrouter {
namespace configuration {

DDSRouterConfiguration::DDSRouterConfiguration(
        std::set<std::shared_ptr<FilterTopic>> allowlist,
        std::set<std::shared_ptr<FilterTopic>> blocklist,
        std::set<std::shared_ptr<RealTopic>> builtin_topics,
        std::set<std::shared_ptr<ParticipantConfiguration>> participants_configurations)
    : allowlist_(allowlist)
    , blocklist_(blocklist)
    , builtin_topics_(builtin_topics)
    , participants_configurations_(participants_configurations)
{
}

std::set<std::shared_ptr<FilterTopic>> DDSRouterConfiguration::allowlist() const noexcept
{
    return allowlist_;
}

std::set<std::shared_ptr<FilterTopic>> DDSRouterConfiguration::blocklist() const noexcept
{
    return blocklist_;
}

std::set<std::shared_ptr<RealTopic>> DDSRouterConfiguration::builtin_topics() const noexcept
{
    return builtin_topics_;
}

std::set<std::shared_ptr<ParticipantConfiguration>> DDSRouterConfiguration::participants_configurations() const noexcept
{
    return participants_configurations_;
}

bool DDSRouterConfiguration::is_valid() const noexcept
{
    // Check Allow list topics
    for (std::shared_ptr<FilterTopic> topic : allowlist_)
    {
        if (!topic->is_valid())
        {
            return false;
        }
    }

    // Check Block list topics
    for (std::shared_ptr<FilterTopic> topic : blocklist_)
    {
        if (!topic->is_valid())
        {
            return false;
        }
    }

    // Check Builtin list topics
    for (std::shared_ptr<RealTopic> topic : builtin_topics_)
    {
        if (!topic->is_valid())
        {
            return false;
        }
    }

    // Check there are at least two participants
    if (participants_configurations_.size() < 2)
    {
        return false;
    }

    // Check Participant Configurations AND
    // check Participant Configuration IDs are not repeated
    std::set<ParticipantId> ids;
    for (std::shared_ptr<ParticipantConfiguration> configuration : participants_configurations_)
    {
        if (!configuration->is_valid())
        {
            return false;
        }
        ids.insert(configuration->id());
    }

    // If the number of ids are not equal the number of configurations, is because they are repeated
    if (ids.size() != participants_configurations_.size())
    {
        return false;
    }

    return true;
}

} /* namespace configuration */
} /* namespace ddsrouter */
} /* namespace eprosima */
