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
 * @file FastPayloadPool.cpp
 *
 */

#include <cpp_utils/exception/UnsupportedException.hpp>
#include <cpp_utils/exception/InconsistencyException.hpp>
#include <cpp_utils/Log.hpp>

#include <ddspipe_core/efficiency/payload/FastPayloadPool.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {

using namespace eprosima::ddspipe::core::types;

bool FastPayloadPool::get_payload(
        uint32_t size,
        Payload& payload)
{
    // Reserve new payload
    if (!reserve_(size, payload))
    {
        return false;
    }

    return true;
}

bool FastPayloadPool::get_payload(
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
        // IMPORTANT: If payload has been reserved from this object, it must have the +4 bytes before data
        // We get this value and add a +1 to set that is referencing from one more payload
        MetaInfoType* reference_place = reinterpret_cast<MetaInfoType*>(src_payload.data);
        reference_place--;

        // Add reference
        (*reference_place)++;

        // Set Payload to refer same payload
        target_payload.data = src_payload.data;
        target_payload.length = src_payload.length;
        target_payload.max_size = src_payload.max_size;
    }
    return true;
}

bool FastPayloadPool::release_payload(
        Payload& payload)
{
    // IMPORTANT: If payload has been reserved from this object, it must have the +4 bytes before data
    // We get this value and add a -1 to set that is referencing from one less payload
    MetaInfoType* reference_place = reinterpret_cast<MetaInfoType*>(payload.data);
    reference_place--;

    // Remove reference
    (*reference_place)--;

    // In case it was the last reference, release payload
    if ((*reference_place) == 0)
    {
        // Release payload
        // NOTE: There is no need to check as release cannot return false
        release_(payload);
    }

    payload.length = 0;
    payload.max_size = 0;
    payload.data = nullptr;
    payload.pos = 0;

    return true;
}

bool FastPayloadPool::reserve_(
        uint32_t size,
        types::Payload& payload)
{
    if (size == 0)
    {
        logDevError(DDSROUTER_PAYLOADPOOL,
                "Trying to reserve a data block of 0 bytes.");
        return false;
    }

    // Allocate memory + 4 bytes for reference
    void* memory_allocated = std::malloc(size + sizeof(MetaInfoType));

    // Use reference space to set that this is referenced for the first time
    MetaInfoType* reference_place = reinterpret_cast<MetaInfoType*>(memory_allocated);
    (*reference_place) = 1;

    payload.data = reinterpret_cast<eprosima::fastrtps::rtps::octet*>(reference_place + 1);
    payload.max_size = size;

    add_reserved_payload_();

    return true;
}

bool FastPayloadPool::release_(
        types::Payload& payload)
{
    // Free memory from the initial allocation, 4 bytes before
    MetaInfoType* reference_place = reinterpret_cast<MetaInfoType*>(payload.data);
    reference_place--;
    free(reference_place);

    // Remove payload internal values
    payload.length = 0;
    payload.max_size = 0;
    payload.data = nullptr;
    payload.pos = 0;

    add_release_payload_();

    return true;
}

} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
