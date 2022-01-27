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

bool DDSRouterConfiguration::is_valid(
        utils::Formatter& error_msg) const noexcept
{
    // Check Allow list topics
    for (std::shared_ptr<FilterTopic> topic : allowlist_)
    {
        if (!topic)
        {
            logError(DDSROUTER_CONFIGURATION, "Invalid ptr in allowlist topics.");
            error_msg << "nullptr Filter Topic in allowlist.";
            return false;
        }

        if (!topic->is_valid())
        {
            error_msg << "Invalid Filter Topic " << topic << " in allowlist.";
            return false;
        }
    }

    // Check Block list topics
    for (std::shared_ptr<FilterTopic> topic : blocklist_)
    {
        if (!topic)
        {
            logError(DDSROUTER_CONFIGURATION, "Invalid ptr in blocklist topics.");
            error_msg << "nullptr Filter Topic in blocklist.";
            return false;
        }

        if (!topic->is_valid())
        {
            error_msg << "Invalid Filter Topic " << topic << " in blocklist.";
            return false;
        }
    }

    // Check Builtin list topics
    for (std::shared_ptr<RealTopic> topic : builtin_topics_)
    {
        if (!topic)
        {
            logError(DDSROUTER_CONFIGURATION, "Invalid ptr in builtin topics.");
            error_msg << "nullptr Topic in builtin.";
            return false;
        }

        if (!topic->is_valid())
        {
            error_msg << "Invalid Topic " << topic << " in Builtin list.";
            return false;
        }
    }

    // Check there are at least two participants
    if (participants_configurations_.size() < 2)
    {
        error_msg << "There must be at least 2 participants.";
        return false;
    }

    // Check Participant Configurations AND
    // check Participant Configuration IDs are not repeated
    std::set<ParticipantId> ids;
    for (std::shared_ptr<ParticipantConfiguration> configuration : participants_configurations_)
    {
        if (!configuration)
        {
            logError(DDSROUTER_CONFIGURATION, "Invalid ptr in participant configurations.");
            error_msg << "nullptr ParticipantConfiguration in participant configurations.";
            return false;
        }

        if (!configuration->is_valid(error_msg))
        {
            error_msg << "Error in Participant " << configuration->id() << ".";
            return false;
        }
        ids.insert(configuration->id());
    }

    // If the number of ids are not equal the number of configurations, is because they are repeated
    if (ids.size() != participants_configurations_.size())
    {
        error_msg << "Participant ids are not unique.";
        return false;
    }

    return true;
}

} /* namespace configuration */
} /* namespace ddsrouter */
} /* namespace eprosima */
