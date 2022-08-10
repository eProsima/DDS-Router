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
 * @file EchoParticipant.hpp
 */

#ifndef __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_ECHOPARTICIPANT_HPP_
#define __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_ECHOPARTICIPANT_HPP_

#include <ddsrouter_core/configuration/participant/EchoParticipantConfiguration.hpp>

#include <participant/implementations/auxiliar/BlankParticipant.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Concrete Participant that prints in stdout each message that arrives.
 */
class EchoParticipant : public BlankParticipant
{
public:

    //! Using parent class constructors
    EchoParticipant(
            std::shared_ptr<configuration::EchoParticipantConfiguration> participant_configuration,
            std::shared_ptr<DiscoveryDatabase> discovery_database);

    //! Override kind() IParticipant method
    types::ParticipantKind kind() const noexcept override;

    //! Print discovery information from endpoint discovered
    void echo_discovery(types::Endpoint endpoint_discovered) const noexcept;

    //! Override create_writer() IParticipant method
    std::shared_ptr<IWriter> create_writer(
            types::RealTopic topic) override;

protected:

    // Deleters do not need to be implemented

    //! Reference to alias access of this object configuration without casting every time
    std::shared_ptr<configuration::EchoParticipantConfiguration> configuration_;

    //! DDS Router shared Discovery Database
    std::shared_ptr<DiscoveryDatabase> discovery_database_;
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_ECHOPARTICIPANT_HPP_ */
