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
 * @file ParticipantsDatabase.cpp
 *
 */

#include <cpp_utils/exception/InconsistencyException.hpp>
#include <cpp_utils/Log.hpp>

#include <ddspipe_core/dynamic/ParticipantsDatabase.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {

ParticipantsDatabase::~ParticipantsDatabase()
{
    // Let the map destroy itself
}

std::shared_ptr<IParticipant> ParticipantsDatabase::get_participant(
        const types::ParticipantId& id) const noexcept
{
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    auto it = participants_.find(id);

    if (it == participants_.end())
    {
        return nullptr;
    }

    return it->second;
}

std::set<types::ParticipantId> ParticipantsDatabase::get_participants_ids() const noexcept
{
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    std::set<types::ParticipantId> result;
    for (auto it : participants_)
    {
        result.insert(it.first);
    }
    return result;
}

std::set<types::ParticipantId> ParticipantsDatabase::get_rtps_participants_ids() const noexcept
{
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    std::set<types::ParticipantId> result;
    for (auto it : participants_)
    {
        if (it.second->is_rtps_kind())
        {
            result.insert(it.first);
        }
    }
    return result;
}

const std::map<types::ParticipantId, std::shared_ptr<IParticipant>>& ParticipantsDatabase::get_participants_map() const noexcept
{
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    return participants_;
}

bool ParticipantsDatabase::empty() const noexcept
{
    return participants_.empty();
}

size_t ParticipantsDatabase::size() const noexcept
{
    return participants_.size();
}

void ParticipantsDatabase::add_participant(
        const types::ParticipantId& id,
        const std::shared_ptr<IParticipant>& participant)
{
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);

    auto it = participants_.find(id);
    if (it != participants_.end())
    {
        throw utils::InconsistencyException(
                  utils::Formatter() << "Participant with Id " << id << " already in database.");
    }
    else
    {
        logInfo(DDSROUTER_PARTICIPANT_DATABASE, "Inserting a new Participant " << id);
    }

    participants_[id] = participant;
}

} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
