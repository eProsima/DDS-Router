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
 * @file BaseWriter.hpp
 */

#ifndef __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_AUXILIAR_BASEWRITER_HPP_
#define __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_AUXILIAR_BASEWRITER_HPP_

#include <atomic>
#include <mutex>

#include <ddsrouter_core/types/participant/ParticipantId.hpp>

#include <writer/IWriter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Abstract Writer that implements generic methods for every Writer.
 *
 * In order to inherit from this class, create the protected method write_ .
 * Implement methods enabled_ and disabled_ in order to give specific functionality to these methods.
 */
class BaseWriter : public IWriter
{
public:

    /**
     * @brief Construct a new Base Writer object
     *
     * @param participant_id id of participant
     * @param topic topic that this Writer will refer to
     * @param payload_pool DDS Router shared PayloadPool
     */
    BaseWriter(
            const types::ParticipantId& participant_id,
            const types::RealTopic& topic,
            std::shared_ptr<PayloadPool> payload_pool);

    /**
     * @brief Set this Writer as enabled
     *
     * It changes the \c enabled_ variable.
     * Call protected method \c enable_() for a specific enable functionality.
     *
     * Override enable() IWriter method
     *
     * Thread safe with mutex \c mutex_ .
     */
    void enable() noexcept override;

    /**
     * @brief Set this Writer as disabled
     *
     * It changes the \c enabled_ variable.
     * Call protected method \c disable_() for a specific disable functionality.
     *
     * Override disable() IWriter method
     *
     * Thread safe with mutex \c mutex_ .
     */
    void disable() noexcept override;

    /**
     * @brief Override write() IWriter method
     *
     * This method calls the protected method \c writer_ to make the actual write function.
     * It only manages the enable/disable status.
     *
     * Thread safe with mutex \c mutex_ .
     */
    virtual utils::ReturnCode write(
            std::unique_ptr<types::DataReceived>& data) noexcept override;

protected:

    /**
     * @brief Do nothing
     *
     * Implement this method for a specific enable functionality.
     *
     * @note This method is called with \c mutex_ locked
     */
    virtual void enable_nts_() noexcept;

    /**
     * @brief Do nothing
     *
     * Implement this method for a specific disable functionality.
     *
     * @note This method is called with \c mutex_ locked
     */
    virtual void disable_nts_() noexcept;

    /**
     * @brief Write method to implement by each Writer implementation
     *
     * Implement this method in every inherited Writer class with write functionality.
     */
    virtual utils::ReturnCode write_(
            std::unique_ptr<types::DataReceived>& data) noexcept  = 0;

    //! Participant parent ID
    types::ParticipantId participant_id_;

    //! Topic that this Writer refers to
    types::RealTopic topic_;

    //! DDS Router shared Payload Pool
    std::shared_ptr<PayloadPool> payload_pool_;

    //! Whether the Writer is currently enabled
    std::atomic<bool> enabled_;

    //! Mutex that guards every access to the Writer
    mutable std::mutex mutex_;

    // Allow operator << to use private variables
    friend std::ostream& operator <<(
            std::ostream&,
            const BaseWriter&);
};

/**
 * @brief \c BaseWriter to stream serialization
 *
 * This method is merely a to_string of a BaseWriter definition.
 * It serialize the ParticipantId and topic
 */
std::ostream& operator <<(
        std::ostream& os,
        const BaseWriter& writer);

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_AUXILIAR_BASEWRITER_HPP_ */
