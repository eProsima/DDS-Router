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

#include <cpp_utils/exception/ConfigurationException.hpp>
#include <cpp_utils/exception/UnsupportedException.hpp>
#include <cpp_utils/Log.hpp>
#include <cpp_utils/utils.hpp>

#include <ddsrouter_core/participants/participant/auxiliar/EchoParticipant.hpp>
#include <ddsrouter_core/participants/participant/rtps/SimpleParticipant.hpp>
#include <ddsrouter_core/participants/participant/rtps/InitialPeersParticipant.hpp>
#include <ddsrouter_core/participants/participant/rtps/DiscoveryServerParticipant.hpp>

#include <ddsrouter_participants/ParticipantFactory.hpp>

namespace eprosima {
namespace ddsrouter {
namespace participants {

using namespace eprosima::ddsrouter::core::types;
using namespace eprosima::ddsrouter::core::configuration;

std::shared_ptr<core::IParticipant> ParticipantFactory::create_participant(
        const ParticipantKind& kind,
        const std::shared_ptr<ParticipantConfiguration>& participant_configuration,
        const std::shared_ptr<core::PayloadPool>& payload_pool,
        const std::shared_ptr<core::DiscoveryDatabase>& discovery_database)
{
    // Create a new Participant depending on the ParticipantKind specified
    switch (kind)
    {
        case ParticipantKind::echo:
            // Echo Participant
        {
            std::shared_ptr<EchoParticipantConfiguration> conf_ =
                    std::dynamic_pointer_cast<EchoParticipantConfiguration>(
                participant_configuration);
            if (!conf_)
            {
                throw utils::ConfigurationException(
                          utils::Formatter() << "Configuration from Participant: " << participant_configuration->id <<
                              " is not for Participant Kind: " << kind);
            }

            return std::make_shared<EchoParticipant> (
                conf_,
                discovery_database);
        }

        case ParticipantKind::simple:
            // Simple RTPS Participant
        {
            std::shared_ptr<SimpleParticipantConfiguration> conf_ =
                    std::dynamic_pointer_cast<SimpleParticipantConfiguration>(
                participant_configuration);
            if (!conf_)
            {
                throw utils::ConfigurationException(
                          utils::Formatter() << "Configuration from Participant: " << participant_configuration->id <<
                              " is not for Participant Kind: " << kind);
            }

            auto participant = std::make_shared<rtps::SimpleParticipant> (
                conf_,
                payload_pool,
                discovery_database);

            // Initialize Participant (this is needed as Participant is not RAII because of Listener)
            participant->init();

            return participant;
        }

        case ParticipantKind::discovery_server:
            // Discovery Server RTPS Participant
        {
            std::shared_ptr<DiscoveryServerParticipantConfiguration> conf_ =
                    std::dynamic_pointer_cast<DiscoveryServerParticipantConfiguration>(
                participant_configuration);
            // TMP: Until Transparency TopicQoS module is available
            if (!conf_)
            {
                throw utils::ConfigurationException(
                          utils::Formatter() << "Configuration from Participant: " << participant_configuration->id << " is not for Participant Kind: " <<
                              kind);
            }

            auto participant =  std::make_shared<rtps::DiscoveryServerParticipant> (
                conf_,
                payload_pool,
                discovery_database);

            // Initialize Participant (this is needed as Participant is not RAII because of Listener)
            participant->init();

            return participant;
        }

        case ParticipantKind::initial_peers:
            // Initial Peers RTPS Participant
        {
            std::shared_ptr<InitialPeersParticipantConfiguration> conf_ =
                    std::dynamic_pointer_cast<InitialPeersParticipantConfiguration>(
                participant_configuration);
            // TMP: Until Transparency TopicQoS module is available
            if (!conf_)
            {
                throw utils::ConfigurationException(
                          utils::Formatter() << "Configuration from Participant: " << participant_configuration->id << " is not for Participant Kind: " <<
                              kind);
            }

            auto participant =  std::make_shared<rtps::InitialPeersParticipant> (
                conf_,
                payload_pool,
                discovery_database);

            // Initialize Participant (this is needed as Participant is not RAII because of Listener)
            participant->init();

            return participant;
        }

        default:
            // This should not happen as every kind must be in the switch
            utils::tsnh(
                utils::Formatter() << "Value of ParticipantKind out of enumeration.");
            return nullptr; // Unreachable code
    }
}

} /* namespace participants */
} /* namespace ddsrouter */
} /* namespace eprosima */
