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

namespace eprosima {
namespace ddsrouter {

// TODO: Add logs

ParticipantDatabase::~ParticipantDatabase()
{
}

void ParticipantDatabase::add_participant(
        ParticipantId id,
        std::shared_ptr<IParticipant> participant)
{
    // TODO: this find is only to check if it exists, decide if needed
    auto it = participants_.find(id);

    if (it != participants_.end())
    {
        // TODO: add warning
    }

    participants_[id] = participant;
}

std::shared_ptr<IParticipant> ParticipantDatabase::get_participant(
        const ParticipantId& id) const
{
    auto it = participants_.find(id);

    if (it != participants_.end())
    {
        return nullptr;
    }

    return it->second;
}

std::vector<ParticipantId> ParticipantDatabase::get_participant_ids() const
{
    std::vector<ParticipantId> result(participants_.size());
    int i = 0;
    for (auto it : participants_)
    {
        // There is already space for all of them, so its added appending
        result[i++] = it.first;
    }
    return result;
}

std::map<ParticipantId, std::shared_ptr<IParticipant>> ParticipantDatabase::get_participant_map() const
{
    return participants_;
}

} /* namespace ddsrouter */
} /* namespace eprosima */
