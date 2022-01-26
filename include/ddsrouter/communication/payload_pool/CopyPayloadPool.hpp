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
 * @file CopyPayloadPool.hpp
 */

#ifndef _DDSROUTER_COMMUNICATION_COPYPAYLOADPOOL_HPP_
#define _DDSROUTER_COMMUNICATION_COPYPAYLOADPOOL_HPP_

#include <ddsrouter/communication/payload_pool/PayloadPool.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * @brief Dummy PayloadPool class to use while efficient one is implemented.
 *
 * This class does not handle references, but copies the payload data in each method required.
 */
class CopyPayloadPool : public PayloadPool
{
    using PayloadPool::PayloadPool;

    bool get_payload(
            uint32_t size,
            Payload& payload) override;

    bool get_payload(
            const Payload& src_payload,
            IPayloadPool*& data_owner,
            Payload& target_payload) override;

    bool release_payload(
            Payload& payload) override;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_COMMUNICATION_COPYPAYLOADPOOL_HPP_ */
