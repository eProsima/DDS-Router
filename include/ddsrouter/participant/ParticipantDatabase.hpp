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
 * @file ParticipantDatabase.hpp
 */

#ifndef _DDSROUTER_PARTICIPANT_PARTICIPANTDATABASE_HPP_
#define _DDSROUTER_PARTICIPANT_PARTICIPANTDATABASE_HPP_

#include <map>
#include <shared_mutex>

#include <ddsrouter/types/participant/ParticipantId.hpp>
#include <ddsrouter/participant/IParticipant.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * @brief Database to store collectively all the Participants of the Router
 *
 * This class will be shared between all Bridges so they have access to the Participants.
 */
class ParticipantDatabase
{
public:

    //! Default constructor for an empty database
    ParticipantDatabase() = default;

    /**
     * Destructor
     *
     * @warning it should not be called if it is not empty
     */
    virtual ~ParticipantDatabase();

    /**
     * @brief Get the participant pointer
     *
     * @param id: id of the participant
     * @return pointer to the participant
     */
    std::shared_ptr<IParticipant> get_participant(
            const ParticipantId& id) const noexcept;

    /**
     * @brief Get all the ids of the participants stored
     *
     * @return list of ids
     */
    std::vector<ParticipantId> get_participant_ids() const noexcept;

    /**
     * @brief Get all the participants stored
     *
     * @return list of pointers to participants indexed by ids
     */
    std::map<ParticipantId, std::shared_ptr<IParticipant>> get_participant_map() const noexcept;

    //! Whether the database is empty
    bool empty() const noexcept;

protected:

    /**
     * @brief Remove the Participant from the database with this id
     *
     * @warning this method should only be called from the DDSRouter
     *
     * @param [in] id: id of the participant to remove
     * @return Pointer to Participant removed
     */
    std::shared_ptr<IParticipant> pop_(const ParticipantId& id) noexcept;

    /**
     * @brief Remove a Participant from the database
     *
     * This method calls \c pop_ with a random id inside the database
     *
     * @param [in] id: id of the participant to remove
     * @return Pointer to Participant removed
     */
    std::shared_ptr<IParticipant> pop_() noexcept;

    /**
     * @brief Add a new participant
     *
     * @warning this method should only be called from the DDSRouter
     *
     * @param [in] id: Id of the new Participant
     * @param [in] participant: Pointer to the new Participant
     */
    void add_participant_(
            ParticipantId id,
            std::shared_ptr<IParticipant> participant) noexcept;

    //! Database variable to store participants pointers indexed by their ids
    std::map<ParticipantId, std::shared_ptr<IParticipant>> participants_;

    //! Mutex to guard access to database
    mutable std::shared_timed_mutex mutex_;

    // DDSRouter should be friend class so it can call add_participant_
    friend class DDSRouter;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_PARTICIPANT_PARTICIPANTDATABASE_HPP_ */
