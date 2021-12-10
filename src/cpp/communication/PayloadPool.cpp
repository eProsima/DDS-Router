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
        logWarning(DDSROUTER_PAYLOADPOOL, "Removing PayloadPool with " << (reserve_count_ - release_count_) <<
                " messages without released.");
    }
    else
    {
        logDebug(DDSROUTER_PAYLOADPOOL,
                "Removing PayloadPool correctly after reserve: " << reserve_count_ << " payloads.");
    }
}

bool PayloadPool::get_payload(
        uint32_t size,
        eprosima::fastrtps::rtps::CacheChange_t& cache_change)
{
    // TODO
    throw UnsupportedException("PayloadPool::get_payload not supported yet");
}

bool PayloadPool::get_payload(
        fastrtps::rtps::SerializedPayload_t& data,
        IPayloadPool*& data_owner,
        eprosima::fastrtps::rtps::CacheChange_t& cache_change)
{
    // TODO
    throw UnsupportedException("PayloadPool::get_payload not supported yet");
}

bool PayloadPool::release_payload(
        eprosima::fastrtps::rtps::CacheChange_t& cache_change)
{
    // TODO
    throw UnsupportedException("PayloadPool::release_payload not supported yet");
}

bool PayloadPool::get_payload(
        uint32_t size,
        Payload& payload)
{
    // TODO
    throw UnsupportedException("PayloadPool::get_payload not supported yet");
}

bool PayloadPool::get_payload(
        const Payload& src_payload,
        Payload& target_payload)
{
    // TODO
    throw UnsupportedException("PayloadPool::get_payload not supported yet");
}

bool PayloadPool::release_payload(
        Payload& payload)
{
    // TODO
    throw UnsupportedException("PayloadPool::release_payload not supported yet");
}

bool PayloadPool::get_payload(
        fastrtps::rtps::SerializedPayload_t& data,
        fastrtps::rtps::CacheChange_t& cache_change)
{
    // TODO
    throw UnsupportedException("PayloadPool::get_payload not supported yet");
}

void PayloadPool::add_reserved_payload_()
{
    ++reserve_count_;
}

void PayloadPool::add_release_payload_()
{
    ++release_count_;
    if (release_count_ > reserve_count_)
    {
        logWarning(DDSROUTER_PAYLOADPOOL,
                "Inconsistent PayloadPool, releasing more payloads than reserved.");
    }
}

} /* namespace ddsrouter */
} /* namespace eprosima */
