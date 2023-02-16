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

#pragma once

#include <atomic>

#include <fastdds/rtps/common/CacheChange.h>
#include <fastdds/rtps/common/SerializedPayload.h>
#include <fastdds/rtps/history/IPayloadPool.h>

#include <ddspipe_core/types/dds/Payload.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {

/**
 * Pool to store and release payloads.
 *
 * A Payload is an object with a blind pointer to a string of bytes and extra info (size, length, etc.).
 * Ideally, any payload data will be copied just once, and move the reference from one element to another.
 * When a payload data has not more references to it, it will be erased.
 *
 * This Pool will be called to store a new message that a Reader receives (ideally it will be called just one
 * per message received).
 * Then, this payload will be moved to the Track. As the payload is already in the pool, there will be no copy.
 * Finally, the payload will be moved to every Writer that has to send the data (ideally without copy).
 */
class PayloadPool : public eprosima::fastrtps::rtps::IPayloadPool
{
public:

    //! Construct an empty PayloadPool
    DDSPIPE_CORE_DllAPI PayloadPool();

    //! Delete PayloadPool and erase every Payload still without release
    DDSPIPE_CORE_DllAPI virtual ~PayloadPool();

    /////
    // FAST DDS PART

    /**
     * @brief Reserve in \c cache_change a new payload of max size \c size .
     *
     * It sets values to the serialized payload inside \c cache_change .
     * This method calls \c get_payload for the serialized payload.
     * The \c cache_change owner is set to \c this .
     *
     * @warning length value in \c payload is not modified.
     *
     * @note This method may reserve new memory.
     *
     * @param [in] size : Size in bytes of the payload that will be reserved
     * @param [out] cache_change : the cache change which SerializedPayload will be set
     *
     * @return true if everything ok
     * @return false if something went wrong
     *
     * @pre Fields @c cache_change must not have the serialized payload initialized.
     */
    DDSPIPE_CORE_DllAPI virtual bool get_payload(
            uint32_t size,
            eprosima::fastrtps::rtps::CacheChange_t& cache_change) override; // TODO add noexcept once is implemented

    /**
     * @brief Store in \c cache_change the \c data payload.
     *
     * This method set \c cache_change serialized payload to the same data in \c data .
     * This method should reuse \c data and not copy it in case the owner of \c data is \c this .
     * It sets values to the serialized payload inside \c cache_change .
     *
     * This method calls \c get_payload for the serialized payload.
     * The \c cache_change owner is set to \c this .
     *
     * @note This method may reserve new memory in case the owner is not \c this .
     *
     * @param [in,out] data          Serialized payload received
     * @param [in,out] data_owner    Payload pool owning incoming data \c data
     * @param [in,out] cache_change  Cache change to assign the payload to
     *
     * @warning @c data_owner can only be changed from @c nullptr to @c this. If a value different from
     * @c nullptr is received it should be left unchanged.
     *
     * @return true if everything ok
     * @return false if something went wrong
     *
     * @pre Fields @c cache_change must not have the serialized payload initialized.
     */
    DDSPIPE_CORE_DllAPI virtual bool get_payload(
            eprosima::fastrtps::rtps::SerializedPayload_t& data,
            IPayloadPool*& data_owner,
            eprosima::fastrtps::rtps::CacheChange_t& cache_change) override; // TODO add noexcept once is implemented

    /**
     * @brief Release the data from the serialized payload inside \c cache_change .
     *
     * @note This method must only release the actual memory of a data in case nobody is referencing it anymore.
     *
     * This method calls \c release_payload for the serialized payload.
     * The \c cache_change owner is set to \c nullptr .
     *
     * @param [in,out] cache_change  Cache change to release the payload from
     *
     * @return true if everything ok
     * @return false if something went wrong
     *
     * @throw IncosistencyException if cache change owner is other than this
     */
    DDSPIPE_CORE_DllAPI virtual bool release_payload(
            eprosima::fastrtps::rtps::CacheChange_t& cache_change) override; // TODO add noexcept once is implemented

    /////
    // DDSROUTER PART

    /**
     * @brief Reserve a new data in \c payload of size \c size.
     *
     * It sets value \c max_size and \c data of \c payload .
     *
     * @note This method may reserve new memory.
     *
     * @warning length value in \c payload is not modified.
     *
     * @param [in] size : Size in bytes of the payload that will be reserved
     * @param [out] payload : the SerializedPayload that will be set
     *
     * @return true if everything ok
     * @return false if something went wrong
     *
     * @pre Fields @c payload must not have been initialized.
     */
    DDSPIPE_CORE_DllAPI virtual bool get_payload(
            uint32_t size,
            types::Payload& payload) = 0;

    /**
     * @brief Store in \c target_payload the data from \c src_payload .
     *
     * This method set \c target_payload fields \c max_size , \c lenght and \c data .
     * This method "should" reuse data in \c src_payload and not copy it in case \c data_owner is \c this .
     *
     * @note This method may reserve new memory in case the owner is not \c this .
     *
     * @param [in] src_payload     Payload to move to target
     * @param [in,out] data_owner      Payload pool owning incoming data \c src_payload
     * @param [out] target_payload  Payload to assign the payload to
     *
     * @warning @c data_owner can only be changed from @c nullptr to @c this. If a value different from
     * @c nullptr is received it must be left unchanged.
     *
     * @return true if everything ok
     * @return false if something went wrong
     *
     * @pre Fields @c target_payload must not have been initialized.
     */
    DDSPIPE_CORE_DllAPI virtual bool get_payload(
            const types::Payload& src_payload,
            IPayloadPool*& data_owner,
            types::Payload& target_payload) = 0;

    /**
     * @brief Release the data from the \c payload .
     *
     * @note This method must only release the actual memory of a data in case nobody is referencing it anymore.
     *
     * @note This method should use method \c reserve_ to reserve new memory.
     *
     * Reset the \c payload info.
     *
     * @param [in,out] payload Payload to release data from
     *
     * @return true if everything ok
     * @return false if something went wrong
     *
     * @pre @c payload must have been initialized from this pool.
     */
    DDSPIPE_CORE_DllAPI virtual bool release_payload(
            types::Payload& payload) = 0;

    //! Wether every payload get has been released.
    DDSPIPE_CORE_DllAPI virtual bool is_clean() const noexcept;

protected:

    /**
     * @brief Reserve a new space of memory for new data.
     *
     * It increases \c reserve_count_ .
     *
     * @param size size of memory chunk to reserve
     * @param payload object where introduce the new data pointer
     *
     * @return true if everything ok
     * @return false if something went wrong
     */
    DDSPIPE_CORE_DllAPI virtual bool reserve_(
            uint32_t size,
            types::Payload& payload);

    /**
     * @brief Free a memory space.
     *
     * It increases \c release_count_ .
     *
     * @param payload object to free the data from
     *
     * @return true if everything ok
     * @return false if something went wrong
     *
     * @throw \c IncosistencyException if more releases than reserves has been done
     */
    DDSPIPE_CORE_DllAPI virtual bool release_(
            types::Payload& payload);

    //! Increase \c reserve_count_
    DDSPIPE_CORE_DllAPI void add_reserved_payload_();

    //! Increase \c release_count_ . Show a warning if there are more releases than reserves.
    DDSPIPE_CORE_DllAPI void add_release_payload_();

    //! Count the number of reserved data from this pool
    std::atomic<uint64_t> reserve_count_;
    //! Count the number of released data from this pool
    std::atomic<uint64_t> release_count_;
};

} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
