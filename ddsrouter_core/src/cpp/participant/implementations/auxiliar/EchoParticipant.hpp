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

#include <participant/implementations/auxiliar/BaseParticipant.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Concrete Participant that prints in stdout each message that arrives.
 */
class EchoParticipant : public BaseParticipant<configuration::ParticipantConfiguration>
{
public:

    //! Using parent class constructors
    using BaseParticipant::BaseParticipant;

protected:

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

#endif /* __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_ECHOPARTICIPANT_HPP_ */
