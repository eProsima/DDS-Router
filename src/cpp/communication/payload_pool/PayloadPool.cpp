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

#include <ddsrouter/communication/payload_pool/PayloadPool.hpp>
#include <ddsrouter/exceptions/InconsistencyException.hpp>
#include <ddsrouter/types/Log.hpp>

namespace eprosima {
namespace ddsrouter {

PayloadPool::PayloadPool()
    : reserve_count_(0)
    , release_count_(0)
{
}

PayloadPool::~PayloadPool()
{
    if (reserve_count_ < release_count_)
    {
        logWarning(DDSROUTER_PAYLOADPOOL, "Removing non Consistent PayloadPool.");
    }
    else if (reserve_count_ != release_count_)
    {
        logWarning(DDSROUTER_PAYLOADPOOL,
                "From " << reserve_count_ << " payloads reserved only " << release_count_ << " has been released.");
    }
    else
    {
        logInfo(DDSROUTER_PAYLOADPOOL,
                "Removing PayloadPool correctly after reserve: " << reserve_count_ << " payloads.");
    }
}

/////
// FAST DDS PART

bool PayloadPool::get_payload(
        uint32_t size,
        eprosima::fastrtps::rtps::CacheChange_t& cache_change)
{
    if (get_payload(size, cache_change.serializedPayload))
    {
        cache_change.payload_owner(this);
        return true;
    }
    else
    {
        logWarning(DDSROUTER_PAYLOADPOOL, "Error occurred while creating payload.")
        return false;
    }
}

bool PayloadPool::get_payload(
        fastrtps::rtps::SerializedPayload_t& data,
        IPayloadPool*& data_owner,
        eprosima::fastrtps::rtps::CacheChange_t& cache_change)
{
    if (get_payload(data, data_owner, cache_change.serializedPayload))
    {
        cache_change.payload_owner(this);
        return true;
    }
    else
    {
        logWarning(DDSROUTER_PAYLOADPOOL, "Error occurred while referencing payload.")
        return false;
    }
}

bool PayloadPool::release_payload(
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
            logWarning(DDSROUTER_PAYLOADPOOL, "Error occurred while releasing payload.")
            return false;
        }
    }
    else
    {
        logError(DDSROUTER_PAYLOADPOOL, "Trying to remove a cache change in an incorrect pool.")
        throw InconsistencyException("Trying to remove a cache change in an incorrect pool.");
    }
}

/////
// INTERNAL PART

void PayloadPool::add_reserved_payload_()
{
    ++reserve_count_;
}

void PayloadPool::add_release_payload_()
{
    ++release_count_;
    if (release_count_ > reserve_count_)
    {
        logError(DDSROUTER_PAYLOADPOOL,
                "Inconsistent PayloadPool, releasing more payloads than reserved.");
        throw InconsistencyException("Inconsistent PayloadPool, releasing more payloads than reserved.");
    }
}

bool PayloadPool::reserve_(
        uint32_t size,
        Payload& payload)
{
    if (size == 0)
    {
        logError(DDSROUTER_PAYLOADPOOL,
                "Trying to reserve a data block of 0 bytes.");
        return false;
    }

    payload.reserve(size);

    add_reserved_payload_();

    return true;
}

bool PayloadPool::release_(
        Payload& payload)
{
    payload.empty();

    add_release_payload_();

    return true;
}

} /* namespace ddsrouter */
} /* namespace eprosima */
