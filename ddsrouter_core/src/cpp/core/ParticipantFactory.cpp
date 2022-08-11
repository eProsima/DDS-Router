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
 * @file ParticipantFactory.cpp
 *
 */

#include <ddsrouter_utils/exception/ConfigurationException.hpp>
#include <ddsrouter_utils/exception/UnsupportedException.hpp>
#include <ddsrouter_utils/Log.hpp>

#include <ddsrouter_core/configuration/participant/ParticipantConfiguration.hpp>
#include <ddsrouter_core/configuration/participant/EchoParticipantConfiguration.hpp>
#include <ddsrouter_utils/utils.hpp>

#include <core/ParticipantFactory.hpp>
#include <participant/implementations/auxiliar/DummyParticipant.hpp>
#include <participant/implementations/auxiliar/EchoParticipant.hpp>
#include <participant/implementations/auxiliar/BlankParticipant.hpp>
#include <participant/implementations/rtps/SimpleParticipant.hpp>
#include <participant/implementations/rtps/InitialPeersParticipant.hpp>
#include <participant/implementations/rtps/DiscoveryServerParticipant.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

using namespace eprosima::ddsrouter::core::types;
using namespace eprosima::ddsrouter::core::configuration;

std::shared_ptr<IParticipant> ParticipantFactory::create_participant(
        std::shared_ptr<configuration::ParticipantConfiguration> participant_configuration,
        std::shared_ptr<PayloadPool> payload_pool,
        std::shared_ptr<DiscoveryDatabase> discovery_database)
{
    // Create a new Participant depending on the ParticipantKind specified by the configuration
    switch (participant_configuration->kind)
    {
        case ParticipantKind::blank:
            // BlankParticipant
            return std::make_shared<BlankParticipant>(participant_configuration->id);

        case ParticipantKind::dummy:
            // DummyParticipant
            return std::make_shared<DummyParticipant>(participant_configuration, payload_pool, discovery_database);

        case ParticipantKind::echo:
            // Echo Participant
        {
            std::shared_ptr<configuration::EchoParticipantConfiguration> conf_ =
                    std::dynamic_pointer_cast<configuration::EchoParticipantConfiguration>(
                participant_configuration);
            if (!conf_)
            {
                throw utils::ConfigurationException(
                          utils::Formatter() << "Configuration from Participant: " << participant_configuration->id <<
                              " is not for Participant Kind: " << participant_configuration->kind);
            }

            return std::make_shared<EchoParticipant> (
                conf_,
                discovery_database);
        }

        case ParticipantKind::simple_rtps:
            // Simple RTPS Participant
        {
            std::shared_ptr<configuration::SimpleParticipantConfiguration> conf_ =
                    std::dynamic_pointer_cast<configuration::SimpleParticipantConfiguration>(
                participant_configuration);
            if (!conf_)
            {
                throw utils::ConfigurationException(
                          utils::Formatter() << "Configuration from Participant: " << participant_configuration->id <<
                              " is not for Participant Kind: " << participant_configuration->kind);
            }

            return std::make_shared<rtps::SimpleParticipant> (
                conf_,
                payload_pool,
                discovery_database);
        }

        case ParticipantKind::local_discovery_server:
        case ParticipantKind::wan_discovery_server:
            // Discovery Server RTPS Participant
        {
            std::shared_ptr<configuration::DiscoveryServerParticipantConfiguration> conf_ =
                    std::dynamic_pointer_cast<configuration::DiscoveryServerParticipantConfiguration>(
                participant_configuration);
            if (!conf_)
            {
                throw utils::ConfigurationException(
                          utils::Formatter() << "Configuration from Participant: " << participant_configuration->id << " is not for Participant Kind: " <<
                              participant_configuration->kind);
            }

            return std::make_shared<rtps::DiscoveryServerParticipant> (
                conf_,
                payload_pool,
                discovery_database);
        }

        case ParticipantKind::wan_initial_peers:
            // Initial Peers RTPS Participant
        {
            std::shared_ptr<configuration::InitialPeersParticipantConfiguration> conf_ =
                    std::dynamic_pointer_cast<configuration::InitialPeersParticipantConfiguration>(
                participant_configuration);
            if (!conf_)
            {
                throw utils::ConfigurationException(
                          utils::Formatter() << "Configuration from Participant: " << participant_configuration->id << " is not for Participant Kind: " <<
                              participant_configuration->kind);
            }

            return std::make_shared<rtps::InitialPeersParticipant> (
                conf_,
                payload_pool,
                discovery_database);
        }

        case ParticipantKind::invalid:
            throw utils::ConfigurationException(utils::Formatter() << "Kind: " << participant_configuration->kind
                                                                   << " is not a valid participant kind name.");

        default:
            // This should not happen as every kind must be in the switch
            utils::tsnh(
                utils::Formatter() << "Value of ParticipantKind out of enumeration.");
            return nullptr; // Unreachable code
    }
}

void ParticipantFactory::remove_participant(
        std::shared_ptr<IParticipant> participant)
{
    // Currently there are no Participants that requires specific remove functionality.
    // Add it in a switch case if in the future some functionality is required.
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
