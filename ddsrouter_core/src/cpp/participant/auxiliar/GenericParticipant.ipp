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
 * @file GenericParticipant.cpp
 */

#ifndef __SRC_DDSROUTERCORE_PARTICIPANT_AUXILIAR_GENERICPARTICIPANT_IPP_
#define __SRC_DDSROUTERCORE_PARTICIPANT_AUXILIAR_GENERICPARTICIPANT_IPP_

#include <participant/auxiliar/GenericParticipant.hpp>
#include <ddsrouter_core/types/participant/ParticipantId.hpp>
#include <ddsrouter_core/types/topic/Topic.hpp>

#include <writer/auxiliar/GenericWriter.ipp>
#include <reader/auxiliar/GenericReader.ipp>

namespace eprosima {
namespace ddsrouter {
namespace core {

template <types::ParticipantKind PartKind>
std::unique_ptr<IWriter> GenericParticipant<PartKind>::create_writer_(
        const types::RealTopic& topic,
        std::shared_ptr<fastrtps::rtps::IPayloadPool>& payload_pool)
{
    return std::make_unique<GenericWriter<PartKind>>(id_, topic, payload_pool.get());
}

template <types::ParticipantKind PartKind>
std::unique_ptr<IReader> GenericParticipant<PartKind>::create_reader_(
        const types::RealTopic& topic,
        std::shared_ptr<fastrtps::rtps::IPayloadPool>& payload_pool,
        DataForwardQueue& data_forward_queue)
{
    return std::make_unique<GenericReader<PartKind>>(id_, topic, payload_pool.get(), data_forward_queue);
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_PARTICIPANT_AUXILIAR_GENERICPARTICIPANT_IPP_ */
