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

#ifndef _DDSROUTER_COMMUNICATION_TRACK_HPP_
#define _DDSROUTER_COMMUNICATION_TRACK_HPP_

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

#include <ddsrouter/participant/IParticipant.hpp>
#include <ddsrouter/reader/IReader.hpp>
#include <ddsrouter/writer/IWriter.hpp>

namespace eprosima {
namespace ddsrouter {

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
     * In Track construction, there are created a \c IReader for the participant \c source and one
     * \c IWriter for each participant in \c targets .
     * It also creates a new thread that manages the transmission between the reader and the writers.
     *
     * @param topic:    Topic that this Track manages communication
     * @param source:   Participant that will receive the data
     * @param targets:  Map of Participants that will send the data received by \c source indexed by Participant id
     * @param enable:   Whether the \c Track should be initialized as enabled. False by default
     *
     * @throw \c InitializationException in case creation fails.
     */
    Track(
            const RealTopic& topic,
            std::shared_ptr<IReader> reader,
            std::map<ParticipantId, std::shared_ptr<IWriter>>&& writers,
            bool enable = false);

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
    void enable();

    /**
     * Disable Track in case it is enabled. This will cause that data will not be transmitted from
     * source to targets.
     * Does nothing if it is disabled
     *
     * This method does not manage if the data is still arriving to the reader.
     *
     * Thread safe
     */
    void disable();

protected:

    /**
     * Callback that will be called by the reader in case there is available data to be forwarded.
     *
     * This method is sent to the Reader so it could call it when there is new data.
     */
    void data_available();

    /**
     * Callback that will be called when there is no more data available to be forwarded.
     */
    void no_more_data_available_();

    /**
     * Whether this Track is enabled
     *
     * Not Thread safe, call with \c available_data_mutex_ guarded
     */
    bool should_transmit_nts_();

    /**
     * Main function of Track.
     * It waits for data to be available
     * Once there is data available, call \c transmit_ till there is no data to be forwarded, and turn back to read
     *
     * Transmission is not executed in case track must be terminated or is not enabled.
     */
    void transmit_thread_function_();

    /**
     * Take data from the Reader \c source and send this data through every writer in \c targets .
     *
     * When no more data is available, call \c no_more_data_available_ and exit.
     *
     * It could exit without having finished transmitting all the data if track should terminate or track becomes
     * disabled.
     */
    void transmit_nts_();

protected:

    /**
     * Topic that this bridge manages communication
     *
     * @note: This variable is stored but not used
     */
    RealTopic topic_;

    //! Reader that will read data
    std::shared_ptr<IReader> reader_;

    //! Writers that will send data forward
    std::map<ParticipantId, std::shared_ptr<IWriter>> writers_;

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
     * Whether there is currently data available to take from the reader.
     *
     * This variable is protected by \c available_data_mutex_
     */
    std::atomic<bool> is_data_available_;

    /**
     * Thread that will manage the transmision of the data
     */
    std::thread transmit_thread_;

    /**
     * Condition variable to wait for new data available or track termination.
     */
    std::condition_variable available_data_condition_variable_;

    /**
     * Mutex to handle access to condition variable \c available_data_condition_variable_ .
     * Mutex to manage access to variable \c is_data_available_ .
     */
    std::mutex available_data_mutex_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_COMMUNICATION_TRACK_HPP_ */
