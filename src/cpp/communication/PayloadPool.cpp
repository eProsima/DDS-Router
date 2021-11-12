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

PayloadPool::~PayloadPool()
{
    // TODO
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

} /* namespace ddsrouter */
} /* namespace eprosima */
