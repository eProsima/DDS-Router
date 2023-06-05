// Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file DdsRouterConfiguration.cpp
 *
 */

#include <cpp_utils/Log.hpp>

#include <ddspipe_participants/configuration/DiscoveryServerParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/EchoParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/InitialPeersParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/SimpleParticipantConfiguration.hpp>

#include <ddsrouter_core/configuration/DdsRouterConfiguration.hpp>
#include <ddsrouter_core/configuration/DdsRouterReloadConfiguration.hpp>
#include <ddsrouter_core/types/ParticipantKind.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

void DdsRouterConfiguration::reload(
        const DdsRouterReloadConfiguration& new_configuration)
{
    this->allowlist = new_configuration.allowlist;
    this->blocklist = new_configuration.blocklist;
}

bool DdsRouterConfiguration::is_valid(
        utils::Formatter& error_msg) const noexcept
{
    // Check Allow list topics
    if (!DdsRouterReloadConfiguration::is_valid(error_msg))
    {
        return false;
    }

    // Check there are at least two participants
    if (participants_configurations.size() < 1)
    {
        error_msg << "There must be at least 1 participant.";
        return false;
    }

    // Check Participant Configurations AND
    // check Participant Configuration IDs are not repeated
    std::set<ddspipe::core::types::ParticipantId> ids;
    for (std::pair<types::ParticipantKind, std::shared_ptr<ddspipe::participants::ParticipantConfiguration>>
            configuration : participants_configurations)
    {
        // Check configuration is not null
        if (!configuration.second)
        {
            logDevError(DDSROUTER_CONFIGURATION, "Invalid ptr in participant configurations.");
            error_msg << "nullptr ParticipantConfiguration in participant configurations. ";
            return false;
        }

        // Check configuration is valid
        if (!configuration.second->is_valid(error_msg))
        {
            error_msg << "Error in Participant " << configuration.second->id << ". ";
            return false;
        }

        // Check that the configuration is of type required
        if (!check_correct_configuration_object_(configuration))
        {
            error_msg
                << "Participant " << configuration.second->id
                << " is not of correct Configuration class " << configuration.first << ".";
            return false;
        }

        // Store every id in a set to see if there are repetitions
        ids.insert(configuration.second->id);
    }

    // If the number of ids are not equal the number of configurations, is because they are repeated
    if (ids.size() != participants_configurations.size())
    {
        error_msg << "Participant ids are not unique. ";
        return false;
    }

    return true;
}

template <typename T>
bool check_correct_configuration_object_by_type_(
        const std::shared_ptr<ddspipe::participants::ParticipantConfiguration> configuration)
{
    return nullptr != std::dynamic_pointer_cast<T>(configuration);
}

bool DdsRouterConfiguration::check_correct_configuration_object_(
        const std::pair<types::ParticipantKind,
        std::shared_ptr<ddspipe::participants::ParticipantConfiguration>> configuration)
{
    switch (configuration.first)
    {
        case types::ParticipantKind::simple:
            return check_correct_configuration_object_by_type_<ddspipe::participants::SimpleParticipantConfiguration>(
                configuration.second);

        case types::ParticipantKind::discovery_server:
            return check_correct_configuration_object_by_type_<ddspipe::participants::DiscoveryServerParticipantConfiguration>(
                configuration.second);

        case types::ParticipantKind::initial_peers:
            return check_correct_configuration_object_by_type_<ddspipe::participants::InitialPeersParticipantConfiguration>(
                configuration.second);

        case types::ParticipantKind::echo:
            return check_correct_configuration_object_by_type_<ddspipe::participants::EchoParticipantConfiguration>(
                configuration.second);

        default:
            return check_correct_configuration_object_by_type_<ddspipe::participants::ParticipantConfiguration>(
                configuration.second);
    }
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
