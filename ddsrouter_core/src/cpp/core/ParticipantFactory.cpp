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
 * @file ParticipantFactory.cpp
 *
 */

#include <cpp_utils/exception/ConfigurationException.hpp>
#include <cpp_utils/utils.hpp>

#include <ddspipe_core/dynamic/DiscoveryDatabase.hpp>
#include <ddspipe_core/dynamic/ParticipantsDatabase.hpp>
#include <ddspipe_core/efficiency/payload/PayloadPool.hpp>
#include <ddspipe_core/interface/IParticipant.hpp>
#include <ddspipe_participants/configuration/DiscoveryServerParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/EchoParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/InitialPeersParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/ParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/SimpleParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/XmlParticipantConfiguration.hpp>
#include <ddspipe_participants/participant/auxiliar/EchoParticipant.hpp>
#include <ddspipe_participants/participant/dds/DiscoveryServerParticipant.hpp>
#include <ddspipe_participants/participant/dds/InitialPeersParticipant.hpp>
#include <ddspipe_participants/participant/dds/SimpleParticipant.hpp>
#include <ddspipe_participants/participant/dds/XmlParticipant.hpp>

#include <ddsrouter_core/types/ParticipantKind.hpp>
#include <ddsrouter_core/core/ParticipantFactory.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

template <typename ConfigurationT, typename ParticipantT, typename ... Args>
std::shared_ptr<ddspipe::core::IParticipant>
generic_create_participant(
        const types::ParticipantKind& kind,
        const std::shared_ptr<ddspipe::participants::ParticipantConfiguration>& participant_configuration,
        Args... args)
{
    participant_configuration->app_id = "DDS_ROUTER";
    participant_configuration->app_metadata = "";

    std::shared_ptr<ConfigurationT> conf_ =
            std::dynamic_pointer_cast<ConfigurationT>(
        participant_configuration);

    if (!conf_)
    {
        throw utils::ConfigurationException(
                  utils::Formatter() << "Configuration from Participant: " << participant_configuration->id <<
                      " is not for Participant Kind: " << kind);
    }

    return std::make_shared<ParticipantT> (
        conf_,
        args ...);
}

template <typename ConfigurationT, typename ParticipantT, typename ... Args>
std::shared_ptr<ddspipe::core::IParticipant>
generic_create_participant_with_init(
        const types::ParticipantKind& kind,
        const std::shared_ptr<ddspipe::participants::ParticipantConfiguration>& participant_configuration,
        Args... args)
{
    auto part = generic_create_participant<ConfigurationT, ParticipantT>(kind, participant_configuration, args ...);
    auto specific_part = static_cast<ParticipantT*>(part.get());
    specific_part->init();
    return part;
}

std::shared_ptr<ddspipe::core::IParticipant> ParticipantFactory::create_participant(
        const types::ParticipantKind& kind,
        const std::shared_ptr<ddspipe::participants::ParticipantConfiguration>& participant_configuration,
        const std::shared_ptr<ddspipe::core::PayloadPool>& payload_pool,
        const std::shared_ptr<ddspipe::core::DiscoveryDatabase>& discovery_database)
{
    // Create a new Participant depending on the ParticipantKind specified by the configuration
    switch (kind)
    {
        case types::ParticipantKind::echo:
            return generic_create_participant<
                ddspipe::participants::EchoParticipantConfiguration,
                ddspipe::participants::EchoParticipant>
                   (
                kind,
                participant_configuration,
                discovery_database
                   );

        case types::ParticipantKind::simple:
            return generic_create_participant_with_init<
                ddspipe::participants::SimpleParticipantConfiguration,
                ddspipe::participants::dds::SimpleParticipant>
                   (
                kind,
                participant_configuration,
                payload_pool,
                discovery_database
                   );

        case types::ParticipantKind::discovery_server:
            return generic_create_participant_with_init<
                ddspipe::participants::DiscoveryServerParticipantConfiguration,
                ddspipe::participants::dds::DiscoveryServerParticipant>
                   (
                kind,
                participant_configuration,
                payload_pool,
                discovery_database
                   );

        case types::ParticipantKind::initial_peers:
            return generic_create_participant_with_init<
                ddspipe::participants::InitialPeersParticipantConfiguration,
                ddspipe::participants::dds::InitialPeersParticipant>
                   (
                kind,
                participant_configuration,
                payload_pool,
                discovery_database
                   );

        case types::ParticipantKind::xml:
            return generic_create_participant_with_init<
                ddspipe::participants::XmlParticipantConfiguration,
                ddspipe::participants::dds::XmlParticipant>
                   (
                kind,
                participant_configuration,
                payload_pool,
                discovery_database
                   );

        default:
            // This should not happen as every kind must be in the switch
            utils::tsnh(
                utils::Formatter() << "Value of ParticipantKind out of enumeration.");
            return nullptr; // Unreachable code
    }
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
