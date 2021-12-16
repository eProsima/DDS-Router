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

#include <ddsrouter/communication/PayloadPool.hpp>
#include <ddsrouter/exceptions/UnsupportedException.hpp>

namespace eprosima {
namespace ddsrouter {

bool CopyPayloadPool::release_payload(
        fastrtps::rtps::CacheChange_t& cache_change)
{
    if (cache_change.payload_owner() == this)
    {
        if (release_payload(cache_change.serializedPayload))
        {
            cache_change.payload_owner(nullptr);
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}

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
        Payload& target_payload)
{
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

bool CopyPayloadPool::get_payload(
        Payload& src_payload,
        eprosima::fastrtps::rtps::CacheChange_t& target_cache_change)
{
    if (!get_payload(src_payload, target_cache_change.serializedPayload))
    {
        return false;
    }
    target_cache_change.payload_owner(this);
    return true;
}

} /* namespace ddsrouter */
} /* namespace eprosima */
