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

#pragma once

#include <atomic>
#include <map>
#include <mutex>

#include <ddspipe_core/efficiency/payload/PayloadPool.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {

/**
 * @brief PayloadPool class to efficiently reuse data pointers get from this pool.
 *
 * It implements zero copy data transmission for payloads get from this pool.
 * It does not handle limit of pools or sizes.
 * It reserves new memory each time is required, and free memory when no one is refereing it anymore.
 *
 * This class stores every data that has been reserved and holds a counter to how many references has.
 * Each get increases the counter. Each release decreases the counter.
 * When the counter reaches 0, the data is freed.
 * The data is indexed by the value of the pointer of the data reserved.
 */
class MapPayloadPool : public PayloadPool
{
public:

    //! Use parent constructor
    using PayloadPool::PayloadPool;

    //! Destroy pool and release every data that has not been released yet.
    ~MapPayloadPool();

    /**
     * @brief Reserve new memory of size \c size for this payload.
     *
     * Add a new entrance in \c reserved_payloads_ with the new data created and set the counter to 1.
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
     * @brief Set \c target_payload data to \c src_payload .
     *
     * In case \c data_owner is \c this , \c target_payload points to the same data as \c src_payload , saving
     * a reserve and a copy, and increase the reference counter.
     * Otherwise, new data is reserved and the data is copied to \c target_payload .
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
     * @brief Decrease reference counter for data in \c payload .
     *
     * Decreases the data inside \c payload in 1.
     * If this was the last payload that was referencing the data, this is released.
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

    //! Store every data reserved and the number of payloads that currently reference it.
    std::map<types::PayloadUnit*, uint32_t> reserved_payloads_;

    //! Guards access to \c reserved_payloads_
    std::mutex reserved_payloads_mutex_;
};

} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
