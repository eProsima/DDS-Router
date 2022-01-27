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
#include <ddsrouter/configuration/participant/DiscoveryServerParticipantConfiguration.hpp>
#include <ddsrouter/configuration/participant/ParticipantConfiguration.hpp>
#include <ddsrouter/configuration/participant/SimpleParticipantConfiguration.hpp>
#include <ddsrouter/types/Log.hpp>
#include <ddsrouter/types/participant/ParticipantKind.hpp>
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
    : DDSRouterReloadConfiguration (allowlist, blocklist, builtin_topics)
    , participants_configurations_(participants_configurations)
{
}

std::set<std::shared_ptr<ParticipantConfiguration>> DDSRouterConfiguration::participants_configurations() const noexcept
{
    return participants_configurations_;
}

bool DDSRouterConfiguration::is_valid(
        utils::Formatter& error_msg) const noexcept
{
    // Check Allow list topics
    if (!DDSRouterReloadConfiguration::is_valid(error_msg))
    {
        return false;
    }

    // Check there are at least two participants
    if (participants_configurations_.size() < 2)
    {
        error_msg << "There must be at least 2 participants. ";
        return false;
    }

    // Check Participant Configurations AND
    // check Participant Configuration IDs are not repeated
    std::set<ParticipantId> ids;
    for (std::shared_ptr<ParticipantConfiguration> configuration : participants_configurations_)
    {
        // Check configuration is not null
        if (!configuration)
        {
            logError(DDSROUTER_CONFIGURATION, "Invalid ptr in participant configurations.");
            error_msg << "nullptr ParticipantConfiguration in participant configurations. ";
            return false;
        }

        // Check configuration is valid
        if (!configuration->is_valid(error_msg))
        {
            error_msg << "Error in Participant " << configuration->id() << ". ";
            return false;
        }

        // TODO: check that the configuration is of type required
        if (!check_correct_configuration_object_(configuration))
        {
            error_msg << "Participant " << configuration->id() << " is not of correct Configuration class. ";
            return false;
        }

        // Store every id in a set to see if there are repetitions
        ids.insert(configuration->id());
    }

    // If the number of ids are not equal the number of configurations, is because they are repeated
    if (ids.size() != participants_configurations_.size())
    {
        error_msg << "Participant ids are not unique. ";
        return false;
    }

    return true;
}

void DDSRouterConfiguration::reload(const DDSRouterReloadConfiguration& new_configuration)
{
    this->allowlist_ = new_configuration.allowlist();
    this->blocklist_ = new_configuration.blocklist();
    this->builtin_topics_ = new_configuration.builtin_topics();
}

template <typename T>
bool check_correct_configuration_object_by_type_(const std::shared_ptr<ParticipantConfiguration> configuration)
{
    return nullptr != std::dynamic_pointer_cast<T>(configuration);
}

bool DDSRouterConfiguration::check_correct_configuration_object_(
    const std::shared_ptr<ParticipantConfiguration> configuration)
{
    switch (configuration->kind()())
    {
    case ParticipantKind::SIMPLE_RTPS:
        return check_correct_configuration_object_by_type_<SimpleParticipantConfiguration>(configuration);

    case ParticipantKind::LOCAL_DISCOVERY_SERVER:
    case ParticipantKind::WAN:
        return check_correct_configuration_object_by_type_<DiscoveryServerParticipantConfiguration>(configuration);

    default:
        return check_correct_configuration_object_by_type_<ParticipantConfiguration>(configuration);
    }
}

} /* namespace configuration */
} /* namespace ddsrouter */
} /* namespace eprosima */
