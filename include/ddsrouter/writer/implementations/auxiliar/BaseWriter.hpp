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

#ifndef _DDSROUTER_WRITER_IMPLEMENTATIONS_AUX_BASEWRITER_HPP_
#define _DDSROUTER_WRITER_IMPLEMENTATIONS_AUX_BASEWRITER_HPP_

#include <atomic>
#include <mutex>

#include <ddsrouter/writer/IWriter.hpp>
#include <ddsrouter/types/participant/ParticipantId.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * Writer that has an empty implementation.
 * It does not send anything.
 */
class BaseWriter : public IWriter
{
public:

    /**
     * @brief Construct a new Base Writer object
     *
     * @param participant_id parent participant id
     * @param topic topic that this Writer will refer to
     * @param payload_pool DDS Router shared PayloadPool
     */
    BaseWriter(
        const ParticipantId& participant_id,
        const RealTopic& topic,
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
    virtual ReturnCode write(
            std::unique_ptr<DataReceived>& data) noexcept override;

protected:

    /**
     * @brief Do nothing
     *
     * Implement this method for a specific enable functionality.
     */
    virtual void enable_() noexcept;

    /**
     * @brief Do nothing
     *
     * Implement this method for a specific disable functionality.
     */
    virtual void disable_() noexcept;

    /**
     * @brief Write method to implement by each Writer implementation
     *
     * Implement this method in every inherited Writer class with write functionality.
     */
    virtual ReturnCode write_(
            std::unique_ptr<DataReceived>& data) noexcept  = 0;

    //! Participant parent ID
    ParticipantId participant_id_;

    //! Topic that this Reader refers to
    RealTopic topic_;

    //! DDS Router shared Payload Pool
    std::shared_ptr<PayloadPool> payload_pool_;

    //! Whether the Reader is currently enabled
    std::atomic<bool> enabled_;

    //! Mutex that guards every access to the Reader
    mutable std::recursive_mutex mutex_;

    // Allow operator << to use private variables
    friend std::ostream& operator <<(std::ostream&, const BaseWriter&);
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

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_WRITER_IMPLEMENTATIONS_AUX_BASEWRITER_HPP_ */
