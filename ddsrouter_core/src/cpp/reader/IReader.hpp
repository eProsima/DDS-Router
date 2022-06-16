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
 * @file IReader.hpp
 */

#ifndef __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_AUXILIAR_BASEREADER_HPP_
#define __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_AUXILIAR_BASEREADER_HPP_

#include <atomic>
#include <mutex>
#include <vector>

#include <writer/IWriter.hpp>
#include <ddsrouter_core/types/endpoint/BaseWriterReader.hpp>
#include <ddsrouter_utils/ReturnCode.hpp>

namespace eprosima {
namespace fastrtps {
namespace rtps {

class IPayloadPool;

} /* namespace rtps */
} /* namespace fastrtps */
} /* namespace eprosima */

namespace eprosima {
namespace ddsrouter {
namespace core {

class DataForwardQueue;

/**
 * Base Reader that implements common methods for every Reader.
 *
 * Exposes enable() and disable() methods that express whether the given topic is enabled or not for this participant.
 * Exposes register_writer() method to record the writers associated to this reader
 *
 * In order to inherit from this class:
 * Implement public take_and_forward() method in derived specializations.
 * Optionally implement protected methods enable_ and disable_ in derived specializations.
 */
class IReader : public types::BaseWriterReader<types::EndpointKind::reader>
{
public:

    /**
     * @brief Construct a new Base Reader object
     *
     * @param participant ID
     * @param topic topic that this Reader will refer to
     * @param payload_pool DDS Router shared IPayloadPool
     * @param data_forward_queue storing data forward tasks
     */
    IReader(
            const types::ParticipantId& id,
            const types::RealTopic& topic,
            fastrtps::rtps::IPayloadPool* payload_pool,
            DataForwardQueue& data_forward_queue);

    virtual ~IReader();

    /**
     * @brief Register a Writer
     *
     * Incorporate this Writer in the list of writers to which messages received in this Reader are forwarded to.
     *
     * @param writer Writer to register
     *
     * @throw InconsistencyException if input writer has already been introduced
     * @throw InconsistencyException if input writer has Participant ID different than this Reader's
     */
    void register_writer(
            IWriter* writer);

    /**
     * @brief Set this Reader as enabled if previously disabled
     *
     * It changes the \c enabled_ variable.
     * Call protected method \c enable_() for a specific enable functionality.
     *
     * Thread safe with CAS pattern .
     */
    utils::ReturnCode enable() noexcept;

    /**
     * @brief Set this Reader as disabled
     *
     * It changes the \c enabled_ variable.
     * Call protected method \c disable_() for a specific disable functionality.
     *
     * Thread safe with CAS pattern .
     */
    utils::ReturnCode disable() noexcept;

    /**
     * @brief Interface for taking data and forwarding to writers
     *
     * Thread safe with lock-free try-TAS semantics
     */
    virtual void take_and_forward() noexcept = 0;

protected:

    /**
     * @brief Do nothing
     *
     * Implement this method class for a specific enable functionality. Only called from public enable() if enabled_ transition false -> true was effective.
     */
    virtual void enable_() noexcept;

    /**
     * @brief Do nothing
     *
     * Implement this method class for a specific enable functionality. Only called from public disable() if enabled_ transition true -> false was effective.
     */
    virtual void disable_() noexcept;


    //! DDS Router Payload Pool, always outlive readers/writers in DDSRouter
    fastrtps::rtps::IPayloadPool* payload_pool_;

    //! Reference to queue of data forward tasks
    DataForwardQueue& data_forward_queue_;

    //! Writers to which messages received by this reader are forward to
    std::vector<IWriter*> writers_;

    //! Whether the Reader is currently enabled
    std::atomic<bool> enabled_;
};

/**
 * @brief \c IReader to stream serialization
 *
 * This method is merely a to_string of a IReader definition.
 * It serialize the ParticipantId and topic
 */
std::ostream& operator <<(
        std::ostream& os,
        const IReader& reader);

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_AUXILIAR_BASEREADER_HPP_ */
