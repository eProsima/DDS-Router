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

#include <map>
#include <set>
#include <shared_mutex>

#include <ddspipe_core/interface/IParticipant.hpp>
#include <ddspipe_core/types/participant/ParticipantId.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {

/**
 * @brief Database to store collectively all the Participants of the Router
 *
 * This class will be shared between all Bridges so they have access to the Participants.
 */
class ParticipantsDatabase
{
public:

    DDSPIPE_CORE_DllAPI ParticipantsDatabase() = default;

    /**
     * Destructor
     *
     * @warning it should not be called if it is not empty
     */
    DDSPIPE_CORE_DllAPI virtual ~ParticipantsDatabase();

    /**
     * @brief Get the participant pointer
     *
     * @param id: id of the participant
     * @return pointer to the participant
     */
    DDSPIPE_CORE_DllAPI std::shared_ptr<IParticipant> get_participant(
            const types::ParticipantId& id) const noexcept;

    /**
     * @brief Get all the ids of the participants stored
     *
     * @return set of ids
     */
    DDSPIPE_CORE_DllAPI std::set<types::ParticipantId> get_participants_ids() const noexcept;

    /**
     * @brief Get all the ids of the RPTS participants stored
     *
     * @return set of ids
     */
    DDSPIPE_CORE_DllAPI std::set<types::ParticipantId> get_rtps_participants_ids() const noexcept;

    /**
     * @brief Get all the participants stored
     *
     * @return map of pointers to participants indexed by ids
     */
    DDSPIPE_CORE_DllAPI const std::map<types::ParticipantId, std::shared_ptr<IParticipant>>& get_participants_map() const noexcept;

    //! Whether the database is empty
    DDSPIPE_CORE_DllAPI bool empty() const noexcept;

    //! Number of Participants in the Database
    DDSPIPE_CORE_DllAPI size_t size() const noexcept;

    /**
     * @brief Add a new participant
     *
     * @param [in] id: Id of the new Participant
     * @param [in] participant: Pointer to the new Participant
     *
     * @throw \c IncosistentException if participant already exist (duplicated ids)
     */
    DDSPIPE_CORE_DllAPI void add_participant(
            const types::ParticipantId& id,
            const std::shared_ptr<IParticipant>& participant);

protected:

    //! Database variable to store participants pointers indexed by their ids
    std::map<types::ParticipantId, std::shared_ptr<IParticipant>> participants_;

    //! Mutex to guard access to database
    mutable std::shared_timed_mutex mutex_;
};

} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
