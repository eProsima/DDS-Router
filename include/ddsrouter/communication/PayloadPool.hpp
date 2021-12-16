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
 * Pool to store and release payloads.
 *
 * A Payload is a blind pointer to a string of bytes.
 * Ideally, any payload will be copied to disk just once, and moved the reference from one element to another.
 * When a payload has not more references to it, it will be erased from the pool.
 *
 * This Pool will be called to store a new message that a Reader receives (ideally it will be called just one
 * per message received).
 * Then this payload will be moved to the Track. As the payload is already in the pool, there will be no copy.
 * Finally, the payload will be moved to every Writer that has to send the data (ideally without copy).
 */
class PayloadPool : public fastrtps::rtps::IPayloadPool
{
public:

    //! Construct an empty PayloadPool
    PayloadPool();

    //! Delete PayloadPool and erase every Payload still without release
    virtual ~PayloadPool();

    /////
    // FAST DDS PART

    /**
     * @brief Reserve in \c cache_change a new payload of size.
     *
     * @note This method reserves new memory.
     *
     * @param [in] size : Size in bytes of the payload that will be reserved
     * @param [out] cache_change : the cache change which SerializedPayload will be set
     *
     * @return \c true if everything ok
     * @return \c false if something went wrong
     */
    bool get_payload(
            uint32_t size,
            fastrtps::rtps::CacheChange_t& cache_change) override; // TODO add noexcept once is implemented

    /**
     * @brief Store in \c cache_change the \c data payload.
     *
     * @note This method reserves new memory in case \c data_owner is a different PayloadPool than the one for
     * \c data . Otherwise, it just increase the reference
     *
     * @param [in,out] data          Serialized payload received
     * @param [in,out] data_owner    Payload pool owning incoming data
     * @param [in,out] cache_change  Cache change to assign the payload to
     *
     * @return \c true if everything ok
     * @return \c false if something went wrong
     */
    bool get_payload(
            fastrtps::rtps::SerializedPayload_t& data,
            IPayloadPool*& data_owner,
            fastrtps::rtps::CacheChange_t& cache_change) override; // TODO add noexcept once is implemented

    /**
     * @brief Decreases reference to the payload inside \c cache_change .
     *
     * @note If this is the las reference for a Payload, it will release it.
     *
     * @param [in,out] cache_change  Cache change to release the payload to
     *
     * @return \c true if everything ok
     * @return \c false if something went wrong
     */
    bool release_payload(
            fastrtps::rtps::CacheChange_t& cache_change) override; // TODO add noexcept once is implemented

    /////
    // DDSROUTER PART

    /**
     * @brief Reserve a new payload of size.
     *
     * @note This method reserves new memory.
     *
     * @param [in] size : Size in bytes of the payload that will be reserved
     * @param [out] payload : the SerializedPayload that will be set
     *
     * @return \c true if everything ok
     * @return \c false if something went wrong
     */
    virtual bool get_payload(
            uint32_t size,
            Payload& payload);

    /**
     * @brief Increment reference to \c src_payload and reference it from \c target_payload .
     *
     * @param [in] src_payload : the SerializedPayload with the data to be referenced
     * @param [out] target_payload : the SerializedPayload that will be set
     *
     * @return \c true if everything ok
     * @return \c false if something went wrong
     */
    virtual bool get_payload(
            const Payload& src_payload,
            Payload& target_payload);

    /**
     * @brief Decreases reference to the \c payload .
     *
     * @note If this is the las reference for a Payload, it will release it.
     *
     * @param [in,out] payload Payload to release
     *
     * @return \c true if everything ok
     * @return \c false if something went wrong
     */
    virtual bool release_payload(
            Payload& payload);

    /**
     * @brief Store in \c target_cache_change the \c srd_data payload and set this as owner.
     *
     * @param [in] srd_data          Serialized payload received
     * @param [in,out] target_cache_change  Cache change to assign the payload to
     *
     * @return \c true if everything ok
     * @return \c false if something went wrong
     */
    virtual bool get_payload(
            Payload& srd_data,
            fastrtps::rtps::CacheChange_t& target_cache_change);

protected:

    bool reserve_(
            uint32_t size,
            Payload& payload);

    bool release_(
            Payload& payload);

    void add_reserved_payload_();
    void add_release_payload_();

    uint64_t reserve_count_;
    uint64_t release_count_;
};

/**
 * @brief Dummy PayloadPool class to use while efficient one is implemented.
 *
 * This class does not handle references, but copies the payload data in each method required.
 */
class CopyPayloadPool : public PayloadPool
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
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_COMMUNICATION_PAYLOADPOOL_HPP_ */
