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
 * @file Track.hpp
 */

#ifndef __SRC_DDSROUTERCORE_COMMUNICATION_TRACK_HPP_
#define __SRC_DDSROUTERCORE_COMMUNICATION_TRACK_HPP_

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

#include <participant/IParticipant.hpp>
#include <reader/IReader.hpp>
#include <writer/IWriter.hpp>

namespace eprosima {
namespace ddsrouter {
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
     * @param enable:   Whether the \c Track should be initialized as enabled. False by default
     */
    Track(
            const types::RealTopic& topic,
            types::ParticipantId reader_participant_id,
            std::shared_ptr<IReader> reader,
            std::map<types::ParticipantId, std::shared_ptr<IWriter>>&& writers,
            std::shared_ptr<PayloadPool> payload_pool,
            bool enable = false) noexcept;

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
     * reading a data and receiving it at the same time from different threads, and this is a scenario that
     * could happen with this design.
     *
     * In order to avoid this deadlock, there is a DataAvailableStatus enumeration setting the status
     * of the data taking into account the Listener(listen) update and the Track(read) update.
     *
     * The main point is to not have any mutex taken while take method is called in the Reader, but a mutex could
     * be used to guard the access to the actual Track data available status.
     */
    //! Status of the data available in the Track's Reader
    enum DataAvailableStatus
    {
        NEW_DATA_ARRIVED,   //! Listener has announced that new data has arrived
        TRANSMITTING_DATA,  //! Track is taking data from the Reader, so it could or could not be data
        NO_MORE_DATA,       //! Track has announced that Reader has no more data, and Listener has not notified new data
    };

    /**
     * Callback that will be called by the reader in case there is available data to be forwarded.
     *
     * This method is sent to the Reader so it could call it when there is new data.
     *
     * This method will set the variable \c data_available_status_ to \c NEW_DATA_ARRIVED and awake the transmit thread.
     * If Track is disabled, the callback will be lost.
     */
    void data_available_() noexcept;

    /**
     * @brief Whether there is data waiting to be taken in the Reader
     *
     * The times there is data is when \c data_available_status_ is set as \c NEW_DATA_ARRIVED or \c TRANSMITTING_DATA
     *
     * @return true if there is available data
     * @return false otherwise
     */
    bool is_data_available_() const noexcept;

    /**
     * Callback that will be called when there is no more data available to be forwarded.
     *
     * @note: this method is called from the Track after receiving a NO_DATA from Reader. But during the time to
     * set \c data_available_status_ the Listener could notify new data (it is not possible to guard this
     * behaviour as no shared mutex could be locked in transmit and listen because of FastDDS Reader mutex taken
     * while \c on_data_available callback). If this happens, it should not be set as NO_DATA, but as new data.
     * If this happens, the transmit thread will stop transmit loop, it will arrive to wait and it will automatically
     * exit it as there is actual data to be sent, so there is no case where it gets stopped with new data available.
     */
    void no_more_data_available_() noexcept;

    /**
     * Whether this Track is enabled
     *
     * This method does not lock a mutex as it only acces atomic values to read them.
     */
    bool should_transmit_() noexcept;

    /**
     * Main function of Track.
     * It waits for data to be available
     * Once there is data available, call \c transmit_ till there is no data to be forwarded, and turn back to read
     *
     * Transmission is not executed in case track must be terminated or is not enabled.
     */
    void transmit_thread_function_() noexcept;

    /**
     * Take data from the Reader \c source and send this data through every writer in \c targets .
     *
     * When no more data is available, call \c no_more_data_available_ and exit.
     *
     * It could exit without having finished transmitting all the data if track should terminate or track becomes
     * disabled.
     */
    void transmit_() noexcept;

    /**
     * @brief Id of the Participant of the Reader
     *
     * This id and topic identifies unequivocally a Track
     */
    types::ParticipantId reader_participant_id_;

    /**
     * Topic that this bridge manages communication
     *
     * @note: This variable is only used for log
     */
    types::RealTopic topic_;

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
    std::recursive_mutex track_mutex_;

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
     * \c NEW_DATA_ARRIVED  : Reader Listener has notified that there are new data
     * \c TRANSMITTING_DATA : Track is currently taking data, so there may or may not be data available
     * \c NO_MORE_DATA      : Track has received a NO_DATA from Reader
     *
     * This variable is protected by \c data_available_mutex_
     */
    std::atomic<DataAvailableStatus> data_available_status_;

    /**
     * Condition variable to wait for new data available or track termination.
     */
    std::condition_variable data_available_condition_variable_;

    /**
     * Mutex to handle access to condition variable \c data_available_condition_variable_ .
     * Mutex to manage access to variable \c data_available_status_ .
     */
    std::mutex data_available_mutex_;

    /**
     * Thread that will manage the transmission of the data
     */
    std::thread transmit_thread_;

    /**
     * Mutex to guard while the Track is sending a message.
     */
    std::mutex on_transmission_mutex_;

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
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_COMMUNICATION_TRACK_HPP_ */
