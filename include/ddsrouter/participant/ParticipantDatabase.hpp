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

#include <ddsrouter/types/ParticipantId.hpp>
#include <ddsrouter/participant/IParticipant.hpp>

namespace eprosima {
namespace ddsrouter {

class ParticipantDatabase
{
public:

    ParticipantDatabase() = default;

    virtual ~ParticipantDatabase();

    // WARNING only used by DDSRouter
    void add_participant(
            ParticipantId id,
            std::shared_ptr<IParticipant> participant);

    std::shared_ptr<IParticipant> get_participant(
            const ParticipantId& id) const;

    std::vector<ParticipantId> get_participant_ids() const;

    std::map<ParticipantId, std::shared_ptr<IParticipant>> get_participant_map() const;

protected:

    std::map<ParticipantId, std::shared_ptr<IParticipant>> participants_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_PARTICIPANT_PARTICIPANTDATABASE_HPP_ */
