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
 * @file BaseReader.hpp
 */

#ifndef __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_AUXILIAR_BASEREADER_HPP_
#define __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_AUXILIAR_BASEREADER_HPP_

#include <atomic>
#include <mutex>

#include <ddsrouter_core/types/participant/ParticipantId.hpp>
#include <ddsrouter_core/interface/IReader.hpp>
#include <ddsrouter_core/efficiency/payload/PayloadPool.hpp>

namespace eprosima {
namespace ddsrouter {
namespace participants {

/**
 * Abstract Reader that implements generic methods for every Reader.
 *
 * In order to inherit from this class, create the protected method take_ .
 * Implement methods enabled_ and disabled_ in order to give specific functionality to these methods.
 */
class BaseReader : public core::IReader
{
public:

    // TODO Implement a way that if messages arrive when not enabled, when enable again call on_data_available

    /**
     * @brief Construct a new Base Reader object
     *
     * @param participant_id parent participant id
     * @param topic topic that this Reader will refer to
     * @param payload_pool DDS Router shared PayloadPool
     */
    BaseReader(
            const core::types::ParticipantId& participant_id,
            const core::types::DdsTopic& topic,
            std::shared_ptr<core::PayloadPool> payload_pool);

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
    void enable() noexcept override;

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
    void disable() noexcept override;

    /**
     * @brief Set the on_data_available callback
     *
     * Override set_on_data_available_callback() IReader method
     *
     * Thread safe with mutex \c mutex_ .
     *
     * @param [in] on_data_available_lambda new callback to notify data received
     */
    void set_on_data_available_callback(
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
    void unset_on_data_available_callback() noexcept override;

    /**
     * @brief Override take() IReader method
     *
     * This method calls the protected method \c take_ to make the actual take function.
     * It only manages the enable/disable status.
     *
     * Thread safe with mutex \c mutex_ .
     */
    utils::ReturnCode take(
            std::unique_ptr<core::types::IRoutingData>& data) noexcept override;

    //! Getter of \c participant_id_ attribute
    core::types::ParticipantId participant_id() const noexcept;

    //! Getter of \c topic_ attribute
    core::types::DdsTopic topic() const noexcept;

protected:

    /**
     * @brief Call from inherit objects whenever there are available data to read
     *
     * It calls the \c on_data_available_lambda_
     *
     * Thread safe with mutex \c mutex_ .
     */
    void on_data_available_() const noexcept;

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
            std::unique_ptr<core::types::IRoutingData>& data) noexcept = 0;

    //! Participant parent ID
    core::types::ParticipantId participant_id_;

    //! Topic that this Reader refers to
    core::types::DdsTopic topic_;

    //! DDS Router shared Payload Pool
    std::shared_ptr<core::PayloadPool> payload_pool_;

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
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_AUXILIAR_BASEREADER_HPP_ */
