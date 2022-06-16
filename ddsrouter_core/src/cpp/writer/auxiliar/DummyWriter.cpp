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
 * @file DummyWriter.cpp
 */

#include <writer/auxiliar/DummyWriter.hpp>
#include <writer/auxiliar/GenericWriter.ipp>

namespace eprosima {
namespace ddsrouter {
namespace core {


GenericWriter<types::ParticipantKind::dummy>::GenericWriter(
        const types::ParticipantId& id,
        const types::RealTopic& topic,
        fastrtps::rtps::IPayloadPool* payload_pool)
    : IWriter(id, topic, payload_pool)
    , guid_(make_auxiliar_guid<types::ParticipantKind::dummy>())
{
}

void GenericWriter<types::ParticipantKind::dummy>::write(
        fastrtps::rtps::CacheChange_t* reader_cache_change) noexcept
{
    char buffer[256];
    void* data = reader_cache_change->serializedPayload.data;
    uint32_t size = reader_cache_change->serializedPayload.max_size;

    if (size == 0)
    {
        return;
    }

    std::memcpy(buffer, data, size);
    std::string str(static_cast<const char*>(buffer), size);

    {
        std::lock_guard<std::mutex> lck(mutex_);
        all_received_.push_back(str);
    }
}

unsigned int GenericWriter<types::ParticipantKind::dummy>::get_received() const
{
    return all_received_.size();
}

bool GenericWriter<types::ParticipantKind::dummy>::check_content(
        const std::string& reference_message) const
{
    for (const auto& received : all_received_)
    {
        if (received != reference_message)
        {
            return false;
        }
    }
    return true;
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
