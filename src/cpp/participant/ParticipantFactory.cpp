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

#include <ddsrouter/configuration/participant/ParticipantConfiguration.hpp>
#include <ddsrouter/exceptions/ConfigurationException.hpp>
#include <ddsrouter/exceptions/UnsupportedException.hpp>
#include <ddsrouter/participant/implementations/auxiliar/DummyParticipant.hpp>
#include <ddsrouter/participant/implementations/auxiliar/EchoParticipant.hpp>
#include <ddsrouter/participant/implementations/auxiliar/VoidParticipant.hpp>
#include <ddsrouter/participant/implementations/rtps/SimpleParticipant.hpp>
#include <ddsrouter/participant/implementations/rtps/LocalDiscoveryServerParticipant.hpp>
#include <ddsrouter/participant/implementations/rtps/WANParticipant.hpp>
#include <ddsrouter/participant/ParticipantFactory.hpp>
#include <ddsrouter/types/Log.hpp>
#include <ddsrouter/types/utils.hpp>

namespace eprosima {
namespace ddsrouter {

// TODO: Add logs

std::shared_ptr<IParticipant> ParticipantFactory::create_participant(
        std::shared_ptr<configuration::ParticipantConfiguration> participant_configuration,
        std::shared_ptr<PayloadPool> payload_pool,
        std::shared_ptr<DiscoveryDatabase> discovery_database)
{
    // Create a new Participant depending on the ParticipantKind specified by the configuration
    switch (participant_configuration->kind()())
    {
        case ParticipantKind::VOID:
            // VoidParticipant
            return std::make_shared<VoidParticipant>(participant_configuration->id());

        case ParticipantKind::ECHO:
            // EchoParticipant
            return std::make_shared<EchoParticipant>((*participant_configuration), payload_pool, discovery_database);

        case ParticipantKind::DUMMY:
            // DummyParticipant
            return std::make_shared<DummyParticipant>((*participant_configuration), payload_pool, discovery_database);

        case ParticipantKind::SIMPLE_RTPS:
            // Simple RTPS Participant
        {
            std::shared_ptr<configuration::SimpleParticipantConfiguration> conf_ =
                    std::dynamic_pointer_cast<configuration::SimpleParticipantConfiguration>(
                participant_configuration);
            if (!conf_)
            {
                throw ConfigurationException(
                          utils::Formatter() << "Configuration from Participant: " << participant_configuration->id() << " is not for Participant Kind: " <<
                                participant_configuration->kind());
            }

            return std::make_shared<rtps::SimpleParticipant> (
                (*conf_),
                payload_pool,
                discovery_database);
        }

        case ParticipantKind::LOCAL_DISCOVERY_SERVER:
            // Discovery Server RTPS Participant
        {
            std::shared_ptr<configuration::DiscoveryServerParticipantConfiguration> conf_ =
                    std::dynamic_pointer_cast<configuration::DiscoveryServerParticipantConfiguration>(
                participant_configuration);
            if (!conf_)
            {
                throw ConfigurationException(
                          utils::Formatter() << "Configuration from Participant: " << participant_configuration->id() << " is not for Participant Kind: " <<
                                participant_configuration->kind());
            }

            return std::make_shared<rtps::LocalDiscoveryServerParticipant> (
                (*conf_),
                payload_pool,
                discovery_database);
        }

        case ParticipantKind::WAN:
            // Discovery Server RTPS Participant
        {
            std::shared_ptr<configuration::DiscoveryServerParticipantConfiguration> conf_ =
                    std::dynamic_pointer_cast<configuration::DiscoveryServerParticipantConfiguration>(
                participant_configuration);
            if (!conf_)
            {
                throw ConfigurationException(
                          utils::Formatter() << "Configuration from Participant: " << participant_configuration->id() << " is not for Participant Kind: " <<
                                participant_configuration->kind());
            }

            return std::make_shared<rtps::WANParticipant> (
                (*conf_),
                payload_pool,
                discovery_database);
        }

        case ParticipantKind::PARTICIPANT_KIND_INVALID:
            throw ConfigurationException(utils::Formatter() << "Kind: " << participant_configuration->kind()
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
    switch (participant->kind()())
    {
        default:
            // Rest of participants do not require specific destructor
            break;
    }
}

} /* namespace ddsrouter */
} /* namespace eprosima */
