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
 * @file ParticipantKind.cpp
 *
 */

#include <algorithm>
#include <assert.h>
#include <set>
#include <iostream>

#include <ddsrouter_core/types/participant/ParticipantKind.hpp>
#include <ddsrouter_utils/utils.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

std::ostream& operator <<(
        std::ostream& os,
        ParticipantKind kind)
{
    try
    {

        os << ParticipantKindStrings.at(static_cast<ParticipantKindType>(kind));

    }
    catch (const std::out_of_range& oor)
    {
        utils::tsnh(utils::Formatter() << "Invalid Participant Kind." << static_cast<ParticipantKindType>(kind));
    }
    return os;
}

ParticipantKind participant_kind_from_name(
        std::string participantKindName)
{

    if (participantKindName.size() == 0)
    {
        return ParticipantKind::invalid;
    }

    ParticipantKindType pkId = 0u;
    for (const auto& aliases : ParticipantKindAliases)
    {
        if (std::find(std::cbegin(aliases), std::cend(aliases), participantKindName) != std::cend(aliases))
        {
            // Alias match, since std::find returned iterator before end
            return AllParticipantKinds.at(pkId);
        }
        pkId++;
    }
    return ParticipantKind::invalid;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
