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

#include <cpp_utils/thread_pool/pool/SlotThreadPool.hpp>
#include <cpp_utils/memory/Heritable.hpp>

#include <ddspipe_core/interface/IParticipant.hpp>
#include <ddspipe_core/interface/IReader.hpp>
#include <ddspipe_core/interface/IWriter.hpp>
#include <ddspipe_core/types/topic/dds/DistributedTopic.hpp>
#include <ddspipe_core/efficiency/payload/PayloadPool.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {

/**
 * Track object manages the communication between one \c IReader as entry point of data and N
 * \c IWriter that will send forward the data received.
 */
class Track
{
public:

    /**
     * Track constructor by required values.
     *
     * Track construction creates a new thread that manages the transmission between the reader and the writers.
     *
     * @param topic:    Topic that this Track manages communication
     * @param reader:   Reader that will receive the remote data
     * @param writers:  Map of Writers that will send the data received by \c source indexed by Participant id
     */
    Track(
            const utils::Heritable<types::DistributedTopic>& topic,
            const types::ParticipantId& reader_participant_id,
            const std::shared_ptr<IReader>& reader,
            std::map<types::ParticipantId, std::shared_ptr<IWriter>>&& writers,
            const std::shared_ptr<PayloadPool>& payload_pool,
            const std::shared_ptr<utils::SlotThreadPool>& thread_pool) noexcept;

    /**
     * @brief Destructor
     *
     * It unsets the callback from Reader.
     * It should stop and wait for the transmission thread.
     * It must not destroy any entity as it does not create them.
     */
    virtual ~Track();

    /**
     * Copy method not allowed
     *
     * Track creates in constructor all the inside Endpoints needed, and thus it should not be copied
     */
    void operator =(
            const Track&) = delete;

    /**
     * Enable Track in case it is not enabled
     * Does nothing if it is already enabled
     *
     * Thread safe
     */
    void enable() noexcept;

    /**
     * Disable Track in case it is enabled. This will cause that data will not be transmitted from
     * source to targets.
     * Does nothing if it is disabled
     *
     * This method does not manage if the data is still arriving to the reader.
     *
     * Thread safe
     */
    void disable() noexcept;

protected:

    /*
     * WORKAROUND:
     * A problem has been found in the use of Track within FastDDS Readers:
     * the on_data_available callback is called with the Reader mutex taken, so it may occur a deadlock while
     * reading a data and receiving it at the same time from different threads if on_data_available and read
     * methods share a mutex.
     *
     * In order to avoid this deadlock, there is a DataAvailableStatus enumeration setting the status
     * of the data taking into account the Listener(listen) update and the Track(read) update.
     * This enumeration works as numbers and not as enumeration (could be seen as a collection of constexpr)
     *
     * The main point is to not have to tak any mutex in on_data_available neither in read.
     */
    //! Status of the data available in the Track's Reader
    enum DataAvailableStatus
    {
        no_more_data = 0,               //! Track has announced that Reader has no more data
        transmitting_data = 1,          //! Track is taking data from the Reader, so it could or could not be data
        new_data_arrived = 2 /* >2 */,  //! Listener has announced that new data has arrived
    };

    /**
     * Callback that will be called by the reader in case there is available data to be forwarded.
     *
     * This method is registered in the Reader so it could call it when there is new data.
     *
     * This method will add the variable \c data_available_status_ in \c new_data_arrived .
     * It will emit a task to execute transmit in a different thread if there was no previous thread before.
     */
    void data_available_() noexcept;

    /**
     * Whether this Track is enabled and should not exit.
     *
     * This method does not lock a mutex as it only acces atomic values to read them.
     */
    bool should_transmit_() noexcept;

    /**
     * Take data from the Reader \c source and send this data through every writer in \c targets .
     *
     * When no more data is available, set \c data_available_status_ as \c no_more_data .
     *
     * It could exit without having finished transmitting all the data if track should terminate or track becomes
     * disabled.
     */
    void transmit_() noexcept;

    //! Topic that refers to this Bridge
    const utils::Heritable<ITopic> topic_;

    /**
     * @brief Id of the Participant of the Reader
     *
     * This id and topic identifies unequivocally a Track
     */
    types::ParticipantId reader_participant_id_;

    //! Reader that will read data
    std::shared_ptr<IReader> reader_;

    //! Writers that will send data forward
    std::map<types::ParticipantId, std::shared_ptr<IWriter>> writers_;

    //! Common shared payload pool
    std::shared_ptr<PayloadPool> payload_pool_;

    //! Whether the Track is currently enabled
    std::atomic<bool> enabled_;

    /**
     * Mutex to prevent simultaneous calls to \c enable and/or \c disable .
     * It manages access to variable \c enabled_ .
     */
    std::mutex track_mutex_;

    /////
    // Transmit thread part

    /**
     * Whether the Track must terminate
     *
     * This variable is only set in destruction. It forces \c transmit_thread_ to stop even if it is
     * transmitting data.
     *
     * As it is only set in destruction, it is not protected by any mutex
     */
    std::atomic<bool> exit_;

    /**
     * Current status of the data available
     *
     * There are 3 states:
     * \c 0 no_more_data      : Track has received a NO_DATA from Reader and no data has been set from on_data_available
     * \c 1 transmitting_data : Track is currently taking data, so there may or may not be data available
     * \c >1 new_data_arrived  : Reader Listener has notified that there are new data
     *
     * This variable does not need to be protected as it is atomic.
     */
    std::atomic<unsigned int> data_available_status_;

    /**
     * Mutex to guard while the Track is sending a message so it could not be disabled.
     */
    std::mutex on_transmission_mutex_;

    utils::TaskId transmit_task_id_;

    std::shared_ptr<utils::SlotThreadPool> thread_pool_;

    static const unsigned int MAX_MESSAGES_TRANSMIT_LOOP_;

    // Allow operator << to use private variables
    friend std::ostream& operator <<(
            std::ostream&,
            const Track&);
};

/**
 * @brief \c Track to stream serialization
 *
 * This method is merely a to_string of a Track definition.
 * It serialize the topic and Participant source Id
 */
std::ostream& operator <<(
        std::ostream& os,
        const Track& track);

} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
