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

#ifndef _DATABROKER_COMMUNICATION_PAYLOADPOOL_HPP_
#define _DATABROKER_COMMUNICATION_PAYLOADPOOL_HPP_

#include <fastdds/rtps/common/CacheChange.h>
#include <fastdds/rtps/common/SerializedPayload.h>
#include <fastdds/rtps/history/IPayloadPool.h>

namespace eprosima {
namespace databroker {

/**
 * TODO
 */
class PayloadPool : public eprosima::fastrtps::rtps::IPayloadPool
{
public:

    PayloadPool() = default;

    virtual ~PayloadPool();

    bool get_payload(
            uint32_t size,
            eprosima::fastrtps::rtps::CacheChange_t& cache_change) override;

    bool get_payload(
            fastrtps::rtps::SerializedPayload_t& data,
            IPayloadPool*& data_owner,
            eprosima::fastrtps::rtps::CacheChange_t& cache_change) override;

    bool release_payload(
            eprosima::fastrtps::rtps::CacheChange_t& cache_change) override;
};

} /* namespace databroker */
} /* namespace eprosima */

#endif /* _DATABROKER_COMMUNICATION_PAYLOADPOOL_HPP_ */
