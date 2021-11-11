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

#ifndef _DATABROKER_COMMUNICATION_TRACK_HPP_
#define _DATABROKER_COMMUNICATION_TRACK_HPP_

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

#include <databroker/participant/IDatabrokerParticipant.hpp>
#include <databroker/reader/IDatabrokerReader.hpp>
#include <databroker/writer/IDatabrokerWriter.hpp>

namespace eprosima {
namespace databroker {

/**
 * Track object manage the communication between one \c IDatabrokerReader as entry point of data and Nç
 * \c IDatabrokerWriter that will send forward the data received.
 */
class Track
{
public:

    /**
     * Track constructor by required values.
     *
     * In Track construction, there are created a \c IDatabrokerReader for the participant \c source and one
     * \c IDatabrokerWriter for each participant in \c targets .
     * It also creates a new thread that manage the transmission between the reader and the writers.
     *
     * @param topic:    Topic that this Track manage communication
     * @param source:   Participant that will receive the data
     * @param targets:  Map of Participants that will send the data received by \c source indexed by Participant id
     * @param enable:   Wether the \c Track should be initialized as enabled. False by default
     *
     * In case any inside endpoint creation fails it will throw a \c InitializationException
     */
    Track(
            const RealTopic& topic,
            std::shared_ptr<IDatabrokerReader> reader,
            std::map<ParticipantId, std::shared_ptr<IDatabrokerWriter>>&& writers,
            bool enable = false);

    virtual ~Track();

    /**
     * Copy method not allowed
     *
     * Track creates in constructor all the inside Endponts needed, and thus it should not be copied
     */
    void operator =(
            const Track&) = delete;

    /**
     * Enable Track in case it is not enabled
     * Does nothing if it is already enabled
     *
     * Thread safe
     */
    void enable();

    /**
     * Disable Track in case it is not enabled. This will cause that data will not be transmitted from
     * source to targets.
     * Does nothing if it is disabled
     *
     * This method does not manage if the data is still arriving from the reader.
     *
     * Thread safe
     */
    void disable();

protected:

    /**
     * Callback that will be called by the reader in case there are available data to read.
     *
     * This method is sent to the Reader so it could call it when there are new data.
     */
    void data_available();

    /**
     * Callback that will be called when there are no more data available to read.
     */
    void no_more_data_available_();

    /**
     * Whether this Track is enabled
     *
     * Thread safe
     */
    bool should_transmit_();

    /**
     * Main function of Track.
     * It waits for data to be available
     * Once there are data avaialble, call \c transmit_ till there is no data to read, and turn back to read
     *
     * Transmission is not executed in case track must be terminated or is not enabled.
     */
    void transmit_thread_function_();

    /**
     * Take data from the Reader \c source and send this data through every writer in \c targets .
     *
     * When no more data is available, call \c no_more_data_available_ and exits.
     *
     * It could exit without being finished transmitting all the data if track should terminate or track becomes
     * disabled.
     */
    void transmit_();

protected:

    /**
     * Topic that this bridge manage communication
     *
     * @note: This variable is stored but not used
     */
    RealTopic topic_;

    //! Reader that will read data
    std::shared_ptr<IDatabrokerReader> reader_;

    //! Writers that will send data forward
    std::map<ParticipantId, std::shared_ptr<IDatabrokerWriter>> writers_;

    //! Whether the Track is currently enable
    std::atomic<bool> enabled_;

    /**
     * Mutex to prevent simultaneous calls to \c enable and/or \c disable .
     * It manage acces to variable \c enabled_ .
     */
    std::recursive_mutex track_mutex_;

    /////
    // Transmit thread part

    /**
     * Whether the Track must terminate
     *
     * This variable is only set in destruction. It forces \c transmit_thread_ to stop evevn if it is
     * transmitting data.
     *
     * As it is only set from destructor, it is not protected by any mutex
     */
    std::atomic<bool> exit_;

    /**
     * Whether there are currently data available to take from the reader.
     *
     * This variable is protected by \c available_data_mutex_
     */
    std::atomic<bool> are_data_available_;

    /**
     * Thread that will manage the transmision of the data
     */
    std::thread transmit_thread_;

    /**
     * Condition variable to wait for new data available or track termination.
     */
    std::condition_variable available_data_condition_variable_;

    /**
     * Mutex to handle acces to condition variable \c available_data_condition_variable_ .
     * Mutex to manage acces to variable \c are_data_available_ .
     */
    std::mutex available_data_mutex_;
};

} /* namespace databroker */
} /* namespace eprosima */

#endif /* _DATABROKER_COMMUNICATION_TRACK_HPP_ */
