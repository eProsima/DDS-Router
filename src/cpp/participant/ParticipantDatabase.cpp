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
 * @file ParticipantDatabase.cpp
 *
 */

#include <ddsrouter/participant/ParticipantDatabase.hpp>
#include <ddsrouter/types/Log.hpp>

namespace eprosima {
namespace ddsrouter {

ParticipantDatabase::~ParticipantDatabase()
{
    if (!participants_.empty())
    {
        logWarning(DDSROUTER_PARTICIPANT_DATABASE, "Erasing Participant Database with still Participants in it");
    }
}

std::shared_ptr<IParticipant> ParticipantDatabase::get_participant(
        const ParticipantId& id) const noexcept
{
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    auto it = participants_.find(id);

    if (it == participants_.end())
    {
        return nullptr;
    }

    return it->second;
}

std::vector<ParticipantId> ParticipantDatabase::get_participant_ids() const noexcept
{
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    std::vector<ParticipantId> result(participants_.size());
    int i = 0;
    for (auto it : participants_)
    {
        // There is already space for all of them, so its added without appending
        result[i++] = it.first;
    }
    return result;
}

std::map<ParticipantId, std::shared_ptr<IParticipant>> ParticipantDatabase::get_participant_map() const noexcept
{
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    return participants_;
}

bool ParticipantDatabase::empty() const noexcept
{
    return participants_.empty();
}

void ParticipantDatabase::add_participant_(
        ParticipantId id,
        std::shared_ptr<IParticipant> participant) noexcept
{
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    // TODO: this find is only to check if it exists, decide if needed
    auto it = participants_.find(id);

    if (it != participants_.end())
    {
        logWarning(DDSROUTER_PARTICIPANT_DATABASE, "Inserting a duplicated Participant " << id);
    }
    else
    {
        logInfo(DDSROUTER_PARTICIPANT_DATABASE, "Inserting a new Participant " << id);
    }

    participants_[id] = participant;
}

std::shared_ptr<IParticipant> ParticipantDatabase::pop_(const ParticipantId& id) noexcept
{
    auto it = participants_.find(id);

    if (it == participants_.end())
    {
        // No this participant stored
        return nullptr;
    }

    std::shared_ptr<IParticipant> participant_to_erase = it->second;
    participants_.erase(it);

    logInfo(DDSROUTER_PARTICIPANT_DATABASE, "Poping Participant " << participant_to_erase->id());

    return participant_to_erase;
}

std::shared_ptr<IParticipant> ParticipantDatabase::pop_() noexcept
{
    if (participants_.empty())
    {
        return nullptr;
    }
    else
    {
        return pop_(participants_.begin()->first);
    }
}

} /* namespace ddsrouter */
} /* namespace eprosima */
