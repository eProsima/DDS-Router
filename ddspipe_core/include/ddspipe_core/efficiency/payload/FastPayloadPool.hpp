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
 * @brief This is the data type that is stored within the data allocated with the payload.
 *
 * It uses an atomic value so can be checked and modified in a single atomic operation.
 */
typedef std::atomic<unsigned int> MetaInfoType;

/**
 * This class implements the interface of PayloadPool and fulfilled with it the interface of IPayloadPool from
 * eProsima Fast DDS.
 *
 * This class is used to manage the allocation and release of the payloads used within the Router.
 * The main target is not to copy or alloc data that is already in memory, but to reuse it safely.
 *
 * This implementation uses an idea get from TopicPayloadPool from fastrtps.
 * This is, to alloc more space than required whenever a new payload is needed, and in this extra space (at the
 * beginning of the data) stores the number of references this data has.
 * As long as this number does not reach 0, the data is not deleted.
 *
 * This is a thread safe lock free (except for one atomic check) implementation.
 *
 * @warning this class requires for all the payloads to be released the same times they are retrieved.
 * In case this does not occur, this object does not guarantee that the data will be correctly released.
 *
 * @warning Payloads used within this class must be allocated from this object (in case of \c get_payload it is enough
 * to use the correct \c data_owner as not this one. ) otherwise it will head to undefined behavior.
 */
class FastPayloadPool : public PayloadPool
{
public:

    //! Use parent constructor
    using PayloadPool::PayloadPool;

    /**
     * Reserve a new space for the payload with the size given
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
     * Reserve in \c target_payload the payload in \c src_payload .
     *
     * In case the src has been reserved from this object, the reference counter is increased and no data is copied.
     * Otherwise, this pool alloc new memory and copy the data
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
     * Release a payload that has been reserved from this pool.
     *
     * It decreases the reference counter of the data and if it reaches 0, the data is deleted.
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

    /**
     * @brief Reimplement parent \c reserve_ method
     *
     * In this implementation, the data is allocated along with space for a \c MetaInfoType object
     * that will count the times the payload is referenced
     *
     * @param size size of memory chunk to reserve
     * @param payload object where introduce the new data pointer
     *
     * @return true if everything ok
     * @return false if something went wrong
     */
    virtual bool reserve_(
            uint32_t size,
            types::Payload& payload) override;

    /**
     * @brief Reimplement parent \c release_ method
     *
     * Data must be released taking into account that the data is allocated with space for a \c MetaInfoType object.
     *
     * @param payload object to free the data from
     *
     * @return true if everything ok
     * @return false if something went wrong
     */
    virtual bool release_(
            types::Payload& payload) override;
};

} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
