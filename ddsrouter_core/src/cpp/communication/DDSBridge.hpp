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
 * @file DDSBridge.hpp
 */

#ifndef __SRC_DDSROUTERCORE_COMMUNICATION_DDSBRIDGE_HPP_
#define __SRC_DDSROUTERCORE_COMMUNICATION_DDSBRIDGE_HPP_

#include <mutex>

#include <communication/Bridge.hpp>

#include <communication/Track.hpp>
#include <ddsrouter_core/types/topic/dds/DdsTopic.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Bridge object manages the communication of a \c DdsTopic.
 * It could be seen as a channel of communication as a DDS Topic, whit several Participants that
 * could publish or subscribe in this specific Topic.
 *
 * It contains N \c Tracks that will manage each direction of the communication,
 * being N the number of Participants of this communication channel.
 */
class DDSBridge : public Bridge
{
public:

    /**
     * Bridge constructor by required values
     *
     * In Bridge construction, the inside \c Tracks are created.
     * In Bridge construction, a Writer and a Reader are created for each Participant.
     *
     * @param topic: Topic of which this Bridge manages communication
     * @param participant_database: Collection of Participants to manage communication
     * @param payload_pool: Payload Pool that handles the reservation/release of payloads throughout the DDS Router
     * @param thread_pool: Shared pool of threads in charge of data transmission.
     * @param enable: Whether the Bridge should be initialized as enabled
     *
     * @throw InitializationException in case \c IWriters or \c IReaders creation fails.
     */
    DDSBridge(
            const types::DdsTopic& topic,
            std::shared_ptr<ParticipantsDatabase> participants_database,
            std::shared_ptr<PayloadPool> payload_pool,
            std::shared_ptr<utils::SlotThreadPool> thread_pool,
            bool enable = false);

    /**
     * @brief Destructor
     *
     * Before deleting, it calls \c disable.
     * It deletes all the tracks created and all Writers and Readers.
     */
    virtual ~DDSBridge();

    // virtual void init_();

    /**
     * Copy method not allowed
     *
     * Bridge creates in constructor all the inside Tracks needed, and thus it should not be copied
     */
    void operator =(
            const Track&) = delete;

    /**
     * Enable bridge in case it is not enabled
     * Does nothing if it is already enabled
     *
     * Thread safe
     */
    void enable() noexcept override;

    /**
     * Disable bridge in case it is not enabled
     * Does nothing if it is disabled
     *
     * Thread safe
     */
    void disable() noexcept override;

protected:

    /**
     * Topic of which this Bridge manages communication
     *
     * @note: This variable is only used for log
     */
    const types::DdsTopic topic_;

    /**
     * Inside \c Tracks
     * They are indexed by the Id of the participant that is source
     */
    std::map<types::ParticipantId, std::unique_ptr<Track>> tracks_;

    //! One writer for each Participant, indexed by \c ParticipantId of the Participant the writer belongs to
    std::map<types::ParticipantId, std::shared_ptr<IWriter>> writers_;

    //! One reader for each Participant, indexed by \c ParticipantId of the Participant the reader belongs to
    std::map<types::ParticipantId, std::shared_ptr<IReader>> readers_;

    //! Mutex to prevent simultaneous calls to enable and/or disable
    std::recursive_mutex mutex_;

    // Allow operator << to use private variables
    friend std::ostream& operator <<(
            std::ostream&,
            const DDSBridge&);
};

/**
 * @brief \c DDSBridge to stream serialization
 *
 * This method is merely a to_string of a Bridge definition.
 * It serialize the topic
 */
std::ostream& operator <<(
        std::ostream& os,
        const DDSBridge& bridge);

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_COMMUNICATION_DDSBRIDGE_HPP_ */
