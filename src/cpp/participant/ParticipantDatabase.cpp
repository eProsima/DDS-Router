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
#include <ddsrouter/exceptions/UnsupportedException.hpp>

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
    // TODO
    throw UnsupportedException("ParticipantDatabase::add_participant not supported yet");
}

std::shared_ptr<IParticipant> ParticipantDatabase::get_participant(
        const ParticipantId& id) const
{
    // TODO
    throw UnsupportedException("ParticipantDatabase::get_participant not supported yet");
}

std::vector<ParticipantId> ParticipantDatabase::get_participant_ids() const
{
    // TODO
    throw UnsupportedException("ParticipantDatabase::get_participant not supported yet");
}

std::map<ParticipantId, std::shared_ptr<IParticipant>> ParticipantDatabase::get_participant_map() const
{
    // TODO
    throw UnsupportedException("ParticipantDatabase::get_participant not supported yet");
}

} /* namespace ddsrouter */
} /* namespace eprosima */
