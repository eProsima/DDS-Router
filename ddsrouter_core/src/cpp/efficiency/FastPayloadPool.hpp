// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file FastPayloadPool.hpp
 */

#ifndef __SRC_DDSROUTERCORE_EFFICIENCY_FASTPAYLOADPOOL_HPP_
#define __SRC_DDSROUTERCORE_EFFICIENCY_FASTPAYLOADPOOL_HPP_

#include <atomic>
#include <map>
#include <mutex>

#include <efficiency/PayloadPool.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * TODO
 */
class FastPayloadPool : public PayloadPool
{
public:

    //! Use parent constructor
    using PayloadPool::PayloadPool;

    /**
     * TODO
     *
     * @param size size of the new chunk of data
     * @param payload object to store the new data
     *
     * @return true if everything OK
     * @return false if something went wrong
     */
    bool get_payload(
            uint32_t size,
            types::Payload& payload) override;

    /**
     * TODO
     *
     * @param [in,out] src_payload     Payload to move to target
     * @param [in,out] data_owner      Payload pool owning incoming data \c src_payload
     * @param [in,out] target_payload  Payload to assign the payload to
     *
     * @return true if everything OK
     * @return false if something went wrong
     *
     * @throw utils::InconsistencyException if \c data_owner is \c this but the data in \c src_payload is not from this pool.
     */
    bool get_payload(
            const types::Payload& src_payload,
            IPayloadPool*& data_owner,
            types::Payload& target_payload) override;

    /**
     * TODO
     *
     * @param payload payload to release
     *
     * @return true if everything OK
     * @return false if something went wrong
     *
     * @throw utils::InconsistencyException if the data in \c payload is not from this pool.
     */
    bool release_payload(
            types::Payload& payload) override;

protected:

    virtual bool reserve_(
            uint32_t size,
            types::Payload& payload) override;

    virtual bool release_(
            types::Payload& payload) override;
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_EFFICIENCY_FASTPAYLOADPOOL_HPP_ */
