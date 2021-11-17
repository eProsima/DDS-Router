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
 * @file Bridge.hpp
 */

#ifndef _DDSROUTER_COMMUNICATION_BRIDGE_HPP_
#define _DDSROUTER_COMMUNICATION_BRIDGE_HPP_

#include <mutex>

#include <ddsrouter/communication/Track.hpp>
#include <ddsrouter/participant/IParticipant.hpp>
#include <ddsrouter/participant/ParticipantDatabase.hpp>
#include <ddsrouter/types/ParticipantId.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * Bridge object manages the communication of a DDS Topic (or \c RealTopic ).
 * It could be seen as a channel of communication as a DDS Topic, whit several Participants that
 * could publish or subscribe in this specific Topic.
 *
 * It contains N \c Tracks that will manage each direction of the communication,
 * being N the number of Participants of this communication channel.
 */
class Bridge
{
public:

    /**
     * Bridge constructor by required values
     *
     * In Bridge construction, the inside \c Tracks are created
     *
     * @param topic: Topic of which this Bridge manages communication
     * @param participant_database: Collection of Participants to manage communication
     * @param enable: Whether the Bridge should be initialized as enabled
     *
     * @throw \c InitializationException in case any inside \c Track creation fails.
     */
    Bridge(
            const RealTopic& topic,
            std::shared_ptr<ParticipantDatabase> participant_database,
            bool enable = false);

    //! Destructor
    virtual ~Bridge();

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
    void enable();

    /**
     * Disable bridge in case it is not enabled
     * Does nothing if it is disabled
     *
     * Thread safe
     */
    void disable();

protected:

    /**
     * Topic of which this Bridge manages communication
     *
     * @note: This variable is stored but not used
     */
    const RealTopic topic_;

    /**
     * Collection of Participants to manage communication between
     *
     * @note: This variable is stored but not used
     */
    const std::shared_ptr<ParticipantDatabase> participants_;

    /**
     * Inside \c Tracks
     * They are indexed by the Id of the participant that is source
     */
    std::map<ParticipantId, std::unique_ptr<Track>> tracks_;

    //! One writer for each Participant, indexed by \c ParticipantId of the Participant the writer belongs to
    std::map<ParticipantId, std::shared_ptr<IWriter>> writers_;

    //! One reader for each Participant, indexed by \c ParticipantId of the Participant the reader belongs to
    std::map<ParticipantId, std::shared_ptr<IReader>> readers_;

    //! Whether the Bridge is currently enabled
    bool enabled_;

    //! Mutex to prevent simultaneous calls to enable and/or disable
    std::recursive_mutex mutex_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_COMMUNICATION_BRIDGE_HPP_ */
