// Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <ddspipe_core/types/data/RtpsPayloadData.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {
namespace types {

RtpsPayloadData::~RtpsPayloadData()
{
    // If payload owner exists and payload has size, release it correctly in pool
    if (payload_owner && payload.length > 0)
    {
        payload_owner->release_payload(payload);
    }
}

types::TopicInternalTypeDiscriminator RtpsPayloadData::internal_type_discriminator() const noexcept
{
    return INTERNAL_TOPIC_TYPE_RTPS;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
