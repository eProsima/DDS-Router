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
 * @file EchoDiscoveryParticipant.hpp
 */

#ifndef __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_ECHODISCOVERYPARTICIPANT_HPP_
#define __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_ECHODISCOVERYPARTICIPANT_HPP_

#include <participant/implementations/auxiliar/BaseParticipant.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Concrete Participant that prints in stdout each message that arrives.
 */
class EchoDiscoveryParticipant : public BaseParticipant<configuration::ParticipantConfiguration>
{
public:

    //! Using parent class constructors
    EchoDiscoveryParticipant(
            const configuration::ParticipantConfiguration participant_configuration,
            std::shared_ptr<PayloadPool> payload_pool,
            std::shared_ptr<DiscoveryDatabase> discovery_database);

protected:

    void echo_discovery(types::Endpoint endpoint_discovered) const noexcept;

    //! Override create_writer_() BaseParticipant method
    std::shared_ptr<IWriter> create_writer_(
            types::RealTopic topic) override;

    //! Override create_reader_() BaseParticipant method
    std::shared_ptr<IReader> create_reader_(
            types::RealTopic topic) override;

    // Deleters do not need to be implemented
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_ECHODISCOVERYPARTICIPANT_HPP_ */
