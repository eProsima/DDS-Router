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
 * @file GenericWriter.cpp
 */
#ifndef __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_AUXILIAR_GENERICWRITER_IPP_
#define __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_AUXILIAR_GENERICWRITER_IPP_

#include <fastrtps/rtps/common/Guid.h>
#include <fastdds/rtps/common/CacheChange.h>

#include <writer/auxiliar/GenericWriter.hpp>
#include <reader/IReader.hpp>
#include <ddsrouter_core/types/participant/ParticipantId.hpp>
#include <ddsrouter_utils/Log.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

template <types::ParticipantKind PartKind>
fastrtps::rtps::GUID_t make_auxiliar_guid()
{
    return fastrtps::rtps::GUID_t(
        fastrtps::rtps::GuidPrefix_t(), (static_cast<uint32_t>(PartKind) + 1) * 1000 + AuxiliarWriterEntityId.fetch_add(
            1));
}

template <types::ParticipantKind PartKind>
GenericWriter<PartKind>::GenericWriter(
        const types::ParticipantId& id,
        const types::RealTopic& topic,
        fastrtps::rtps::IPayloadPool* payload_pool)
    : IWriter(id, topic, payload_pool)
    , guid_(make_auxiliar_guid<PartKind>())
{
}

template <types::ParticipantKind PartKind>
void GenericWriter<PartKind>::write(
        fastrtps::rtps::CacheChange_t* reader_cache_change) noexcept
{
    if (PartKind == types::ParticipantKind::echo)   // constexpr

    {
        logInfo(DDSROUTER_WRITER,
                "Echo Participant: " << id_ << " has received from data from: " << reader_cache_change->writerGUID << " in topic: " << topic_ <<
                ">");

    }
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_AUXILIAR_GENERICWRITER_IPP_ */
