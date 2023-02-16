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
#include <mutex>

#include <ddspipe_core/types/participant/ParticipantId.hpp>
#include <ddspipe_core/interface/IReader.hpp>
#include <ddspipe_core/interface/ITopic.hpp>
#include <ddspipe_core/efficiency/payload/PayloadPool.hpp>

#include <ddspipe_participants/library/library_dll.h>

namespace eprosima {
namespace ddspipe {
namespace participants {

/**
 * Abstract Reader that implements generic methods for every Reader.
 *
 * In order to inherit from this class, create the protected method take_ .
 * Implement methods enabled_ and disabled_ in order to give specific functionality to these methods.
 *
 * @todo make topic somehow to can hold it.
 */
class BaseReader : public core::IReader
{
public:

    /////////////////////////
    // CONSTRUCTORS
    /////////////////////////

    // TODO Implement a way that if messages arrive when not enabled, when enable again call on_data_available

    // Protected ctor to make class abstract (only built by their childs).

    /////////////////////////
    // I READER METHODS
    /////////////////////////

    /**
     * @brief Set this Reader as enabled
     *
     * It changes the \c enabled_ variable.
     * Call protected method \c enable_() for a specific enable functionality.
     *
     * Override enable() IReader method
     *
     * Thread safe with mutex \c mutex_ .
     */
    DDSPIPE_PARTICIPANTS_DllAPI void enable() noexcept override;

    /**
     * @brief Set this Reader as disabled
     *
     * It changes the \c enabled_ variable.
     * Call protected method \c disable_() for a specific disable functionality.
     *
     * Override disable() IReader method
     *
     * Thread safe with mutex \c mutex_ .
     */
    DDSPIPE_PARTICIPANTS_DllAPI void disable() noexcept override;

    /**
     * @brief Set the on_data_available callback
     *
     * Override set_on_data_available_callback() IReader method
     *
     * Thread safe with mutex \c mutex_ .
     *
     * @param [in] on_data_available_lambda new callback to notify data received
     */
    DDSPIPE_PARTICIPANTS_DllAPI void set_on_data_available_callback(
            std::function<void()> on_data_available_lambda) noexcept override;

    /**
     * @brief Unset the on_data_available callback
     *
     * Override unset_on_data_available_callback() IReader method
     *
     * Thread safe with mutex \c mutex_ .
     *
     * @param [in] on_data_available_lambda new callback to notify data received
     */
    DDSPIPE_PARTICIPANTS_DllAPI void unset_on_data_available_callback() noexcept override;

    /**
     * @brief Override take() IReader method
     *
     * This method calls the protected method \c take_ to make the actual take function.
     * It only manages the enable/disable status.
     *
     * Thread safe with mutex \c mutex_ .
     */
    DDSPIPE_PARTICIPANTS_DllAPI utils::ReturnCode take(
            std::unique_ptr<core::IRoutingData>& data) noexcept override;

    /////////////////////////
    // AUXILIARY METHODS
    /////////////////////////

    //! Getter of \c participant_id_ attribute
    DDSPIPE_PARTICIPANTS_DllAPI core::types::ParticipantId participant_id() const noexcept override;

    /////////////////////////
    // RPC REQUIRED METHODS
    /////////////////////////
    // TODO remove these methods once the double reference is solved

    //! Get GUID of internal RTPS reader
    DDSPIPE_PARTICIPANTS_DllAPI core::types::Guid guid() const override = 0;

    //! Get internal RTPS reader mutex
    DDSPIPE_PARTICIPANTS_DllAPI fastrtps::RecursiveTimedMutex& get_rtps_mutex() const override = 0;

    //! Get number of unread cache changes in internal RTPS reader
    DDSPIPE_PARTICIPANTS_DllAPI uint64_t get_unread_count() const override = 0;

    DDSPIPE_PARTICIPANTS_DllAPI core::types::DdsTopic topic() const override = 0;
    /////////////////////////

protected:

    /////////////////////////
    // CONSTRUCTORS
    /////////////////////////

    /**
     * @brief Construct a new Base Reader object
     *
     * @param participant_id parent participant id
     * @param topic topic that this Reader will refer to
     * @param payload_pool DDS Router shared PayloadPool
     */
    BaseReader(
            const core::types::ParticipantId& participant_id,
            const std::shared_ptr<core::PayloadPool>& payload_pool);

    /////////////////////////
    // PROTECTED METHODS
    /////////////////////////

    /**
     * @brief Call from inherit objects whenever there are available data to read
     *
     * It calls the \c on_data_available_lambda_
     *
     * Thread safe with mutex \c mutex_ .
     */
    void on_data_available_() const noexcept;

    /////////////////////////
    // METHODS TO IMPLEMENT BY SUBCLASSES
    /////////////////////////

    /**
     * @brief Do nothing
     *
     * Implement this method class for a specific enable functionality.
     */
    virtual void enable_nts_() noexcept;

    /**
     * @brief Do nothing
     *
     * Implement this method class for a specific disable functionality.
     */
    virtual void disable_nts_() noexcept;

    /**
     * @brief Take method to implement by each Reader implementation
     *
     * Implement this method in every inherited Reader class with take functionality.
     */
    virtual utils::ReturnCode take_nts_(
            std::unique_ptr<core::IRoutingData>& data) noexcept = 0;

    /////////////////////////
    // INTERNAL VARIABLES
    /////////////////////////

    //! Participant parent ID
    const core::types::ParticipantId participant_id_;

    //! DDS Router shared Payload Pool
    const std::shared_ptr<core::PayloadPool> payload_pool_;

    //! Lambda to call the callback whenever a new data arrives
    std::function<void()> on_data_available_lambda_;

    //! True if lambda callback is set
    bool on_data_available_lambda_set_;

    //! Whether the Reader is currently enabled
    std::atomic<bool> enabled_;

    //! Mutex that guards every access to the Reader
    mutable std::recursive_mutex mutex_;

    //! Default callback. It shows a warning that callback is not set
    static const std::function<void()> DEFAULT_ON_DATA_AVAILABLE_CALLBACK;

    // Allow operator << to use private variables
    friend std::ostream& operator <<(
            std::ostream&,
            const BaseReader&);
};

/**
 * @brief \c BaseReader to stream serialization
 *
 * This method is merely a to_string of a BaseReader definition.
 * It serialize the ParticipantId and topic
 */
std::ostream& operator <<(
        std::ostream& os,
        const BaseReader& reader);

} /* namespace participants */
} /* namespace ddspipe */
} /* namespace eprosima */
