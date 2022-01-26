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
 * @file MapPayloadPool.hpp
 */

#ifndef _DDSROUTER_COMMUNICATION_MAPPAYLOADPOOL_HPP_
#define _DDSROUTER_COMMUNICATION_MAPPAYLOADPOOL_HPP_

#include <atomic>

#include <ddsrouter/communication/PayloadPool.hpp>

namespace eprosima {
namespace ddsrouter {

struct PayloadNode
{
    std::atomic<uint32_t> ref_counter{ 0 };
    uint32_t data_size = 0;
}

/**
 * @brief PayloadPool class to efficiently reuse those pointers get from this pool.
 *
 * It implements zero copy data transmission for payloads get from this pool.
 *
 * It does not handle limit of pools or sizes.
 */
class MapPayloadPool : public PayloadPool
{

    using PayloadPool::PayloadPool;

    bool release_payload(
            fastrtps::rtps::CacheChange_t& cache_change) override;

    bool get_payload(
            uint32_t size,
            Payload& payload) override;

    bool get_payload(
            const Payload& src_payload,
            Payload& target_payload) override;

    bool release_payload(
            Payload& payload) override;

    bool get_payload(
            Payload& data,
            fastrtps::rtps::CacheChange_t& cache_change) override;

protected:

    std::map<*PayloadUnit, PayloadNode> loaned_payloads_;

};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_COMMUNICATION_MAPPAYLOADPOOL_HPP_ */
