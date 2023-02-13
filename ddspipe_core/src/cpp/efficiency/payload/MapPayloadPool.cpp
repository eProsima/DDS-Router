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
 * @file MapPayloadPool.cpp
 *
 */

#include <cpp_utils/exception/UnsupportedException.hpp>
#include <cpp_utils/exception/InconsistencyException.hpp>
#include <cpp_utils/Log.hpp>

#include <ddspipe_core/efficiency/payload/MapPayloadPool.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {

using namespace eprosima::ddspipe::core::types;

MapPayloadPool::~MapPayloadPool()
{
    if (reserved_payloads_.size() > 0)
    {
        logDevError(
            DDSROUTER_PAYLOADPOOL,
            "Removing MapPayloadPool with still " << reserved_payloads_.size() << " payloads referenced.");

        // Data could not be erased because they will be erased once the Payload is destroyed
    }
}

bool MapPayloadPool::get_payload(
        uint32_t size,
        Payload& payload)
{
    // Reserve new payload
    if (!reserve_(size, payload))
    {
        return false;
    }
    payload.max_size = size;

    // Store this payload in map
    {
        std::lock_guard<std::mutex> lock(reserved_payloads_mutex_);
        reserved_payloads_[payload.data] = 1;
    }

    return true;
}

bool MapPayloadPool::get_payload(
        const Payload& src_payload,
        IPayloadPool*& data_owner,
        Payload& target_payload)
{
    // If we are not the owner, create a new payload. Else, reference the existing one
    if (data_owner != this)
    {
        // Store space for payload
        if (!get_payload(src_payload.max_size, target_payload))
        {
            return false;
        }

        // Copy info
        std::memcpy(target_payload.data, src_payload.data, src_payload.length);
        target_payload.length = src_payload.length;
    }
    else
    {
        std::lock_guard<std::mutex> lock(reserved_payloads_mutex_);

        // src_payload must be inside reserved payloads
        auto payload_it = reserved_payloads_.find(src_payload.data);
        if (payload_it == reserved_payloads_.end())
        {
            logError(DDSROUTER_PAYLOADPOOL, "Payload ownership is this pool, but it is not reserved from here.");
            throw utils::InconsistencyException("Payload ownership is this pool, but it is not reserved from here.");
        }

        // Add reference
        payload_it->second++;

        // Set Payload to refer same payload
        target_payload.data = src_payload.data;
        target_payload.length = src_payload.length;
        target_payload.max_size = src_payload.max_size;
    }
    return true;
}

bool MapPayloadPool::release_payload(
        Payload& payload)
{
    std::lock_guard<std::mutex> lock(reserved_payloads_mutex_);

    // Check that this payload is in this pool
    auto payload_it = reserved_payloads_.find(payload.data);
    if (payload_it == reserved_payloads_.end())
    {
        logError(DDSROUTER_PAYLOADPOOL, "Trying to release a payload from this pool that is not present.");
        throw utils::InconsistencyException("Trying to release a payload from this pool that is not present.");
    }

    // Dereference element
    payload_it->second--;

    // In case it was the last reference, release payload
    if (payload_it->second == 0)
    {
        if (!release_(payload))
        {
            return false;
        }

        // Remove it from map
        reserved_payloads_.erase(payload_it);
    }

    // Restore payload info
    payload.length = 0;
    payload.pos = 0;
    payload.max_size = 0;
    payload.data = nullptr;

    return true;
}

} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
