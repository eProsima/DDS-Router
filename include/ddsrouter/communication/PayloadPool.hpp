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
 * @file PayloadPool.hpp
 */

#ifndef _DDSROUTER_COMMUNICATION_PAYLOADPOOL_HPP_
#define _DDSROUTER_COMMUNICATION_PAYLOADPOOL_HPP_

#include <fastdds/rtps/common/CacheChange.h>
#include <fastdds/rtps/common/SerializedPayload.h>
#include <fastdds/rtps/history/IPayloadPool.h>

#include <ddsrouter/types/Data.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * TODO
 */
class PayloadPool : public fastrtps::rtps::IPayloadPool
{
public:

    PayloadPool() = default;

    virtual ~PayloadPool();

    /////
    // FAST DDS PART

    bool get_payload(
            uint32_t size,
            fastrtps::rtps::CacheChange_t& cache_change) override;

    bool get_payload(
            fastrtps::rtps::SerializedPayload_t& data,
            IPayloadPool*& data_owner,
            fastrtps::rtps::CacheChange_t& cache_change) override;

    bool release_payload(
            fastrtps::rtps::CacheChange_t& cache_change) override;

    /////
    // DDSROUTER PART

    virtual bool get_payload(
            uint32_t size,
            Payload& payload);

    virtual bool get_payload(
            const Payload& src_payload,
            Payload& target_payload);

    virtual bool release_payload(
            Payload& payload);

    virtual bool get_payload(
            fastrtps::rtps::SerializedPayload_t& data,
            fastrtps::rtps::CacheChange_t& cache_change);
};

class CopyPayloadPool : public PayloadPool
{
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
            fastrtps::rtps::SerializedPayload_t& data,
            fastrtps::rtps::CacheChange_t& cache_change) override;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_COMMUNICATION_PAYLOADPOOL_HPP_ */
