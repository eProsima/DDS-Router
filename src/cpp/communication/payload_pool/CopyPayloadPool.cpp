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
 * @file PayloadPool.cpp
 *
 */

#include <ddsrouter/communication/payload_pool/CopyPayloadPool.hpp>
#include <ddsrouter/exceptions/UnsupportedException.hpp>

namespace eprosima {
namespace ddsrouter {

bool CopyPayloadPool::get_payload(
        uint32_t size,
        Payload& payload)
{
    reserve_(size, payload);
    payload.length = size;

    return true;
}

bool CopyPayloadPool::get_payload(
        const Payload& src_payload,
        IPayloadPool*& data_owner,
        Payload& target_payload)
{
    // As this class copies always the data, it does not matter the owner of this data
    static_cast<void>(data_owner);

    if (!get_payload(src_payload.length, target_payload))
    {
        return false;
    }
    std::memcpy(target_payload.data, src_payload.data, src_payload.length);

    return true;
}

bool CopyPayloadPool::release_payload(
        Payload& payload)
{
    return release_(payload);
}

} /* namespace ddsrouter */
} /* namespace eprosima */
