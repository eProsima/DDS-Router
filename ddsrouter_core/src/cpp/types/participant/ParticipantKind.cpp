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
#include <ddsrouter_utils/format/format_utils.hpp>
#include <ddsrouter_utils/format/Formatter.hpp>
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
        os << PARTICIPANT_KIND_STRINGS.at(static_cast<ParticipantKindType>(kind));
    }
    catch (const std::out_of_range& oor)
    {
        utils::tsnh(utils::Formatter() << "Invalid Participant Kind." << static_cast<ParticipantKindType>(kind));
    }
    return os;
}

ParticipantKind participant_kind_from_name(
        std::string kind_str)
{

    // Empty strings are always invalid
    if (kind_str.size() == 0)
    {
        return ParticipantKind::invalid;
    }

    // Convert to lower case so that match is case-insensitive
    utils::to_lowercase(kind_str);

    // Loop over each participant kind aliases, returning at first alias match
    ParticipantKindType kind_idx = 0u;
    for (const auto& aliases : PARTICIPANT_KIND_ALIASES)
    {
        if (std::find_if(std::cbegin(aliases), std::cend(aliases),
                [kind_str](std::string str)
                {
                    utils::to_lowercase(str);
                    return kind_str == str;
                }) != std::cend(aliases))
        {
            // Alias match, since std::find returned iterator before end
            return ALL_PARTICIPANT_KINDS.at(kind_idx);
        }
        kind_idx++;
    }

    // No alias match, so input string is not a valid alias
    return ParticipantKind::invalid;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
