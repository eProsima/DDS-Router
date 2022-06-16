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
 * @file BlankParticipant.hpp
 */

#ifndef __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_GENERICPARTICIPANT_HPP_
#define __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_GENERICPARTICIPANT_HPP_

#include <ddsrouter_core/types/participant/ParticipantId.hpp>
#include <participant/IParticipant.hpp>

namespace eprosima {
namespace fastrtps {
namespace rtps {

class IPayloadPool;

} /* namespace rtps */
} /* namespace fastrtps */
} /* namespace eprosima */

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Participant template exposing a generic interface for creating auxiliar writers and readers.
 *
 * Meant to be used for:
 * ParticipantKind::blank
 * ParticipantKind::echo
 * ParticipantKind::dummy
 */
template <types::ParticipantKind PartKind>
class GenericParticipant : public IParticipant
{
public:

    //! Using parent class constructors
    using IParticipant::IParticipant;

protected:

    //! Override create_writer() IParticipant method
    std::unique_ptr<IWriter> create_writer_(
            const types::RealTopic& topic,
            std::shared_ptr<fastrtps::rtps::IPayloadPool>& payload_pool) override;

    //! Override create_reader() IParticipant method
    std::unique_ptr<IReader> create_reader_(
            const types::RealTopic& topic,
            std::shared_ptr<fastrtps::rtps::IPayloadPool>& payload_pool,
            DataForwardQueue& data_forward_queue) override;
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_GENERICPARTICIPANT_HPP_ */
