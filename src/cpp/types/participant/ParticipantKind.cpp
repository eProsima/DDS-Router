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

#include <ddsrouter/types/participant/ParticipantKind.hpp>
#include <ddsrouter/types/utils.hpp>

namespace eprosima {
namespace ddsrouter {

const std::map<ParticipantKindValue, std::vector<std::string>> ParticipantKind::participant_type_with_aliases_ =
{
    {PARTICIPANT_KIND_INVALID, {"invalid_participant_type"}},
    {VOID, {"void"}},
    {ECHO, {"echo"}},
    {DUMMY, {"dummy"}},
    {SIMPLE_RTPS, {"local", "simple"}},
    {LOCAL_DISCOVERY_SERVER, {"discovery-server", "ds", "local-ds", "local-discovery-server"}},
    {WAN, {"wan", "router"}},
};

ParticipantKind::ParticipantKind(
        ParticipantKindValue value) noexcept
    : value_(value)
{
}

ParticipantKind::ParticipantKind() noexcept
    : ParticipantKind(PARTICIPANT_KIND_INVALID)
{
}

bool ParticipantKind::is_valid() const noexcept
{
    return value_ != PARTICIPANT_KIND_INVALID;
}

std::string ParticipantKind::to_string() const noexcept
{
    auto it = ParticipantKind::participant_type_with_aliases_.find(value_);

    // Value must be in map
    assert(it != ParticipantKind::participant_type_with_aliases_.end());

    return it->second[0];
}

ParticipantKindValue ParticipantKind::operator ()() const noexcept
{
    return value_;
}

bool ParticipantKind::operator <(
        const ParticipantKind& other) const noexcept
{
    return this->value_ < other.value_;
}

bool ParticipantKind::operator ==(
            const ParticipantKind& other) const noexcept
{
    return (*this)() == other();
}

ParticipantKind ParticipantKind:: participant_kind_from_name(
        std::string type) noexcept
{
    // Pass type name to lowercase
    utils::to_lowercase(type);

    // Look for string in names map and return the type it matches
    for (auto types : ParticipantKind::participant_type_with_aliases_)
    {
        // Check if it fix with any of the aliases
        for (std::string alias : types.second)
        {
            if (type == alias)
            {
                return types.first;
            }
        }
    }
    return PARTICIPANT_KIND_INVALID;
}

std::vector<ParticipantKind> ParticipantKind::all_valid_participant_kinds() noexcept
{
    std::vector<ParticipantKind> result;

    for (auto type : participant_type_with_aliases_)
    {
        result.push_back(type.first);
    }

    // Remove invalid type
    result.erase(
        std::remove_if(
            result.begin(),
            result.end(),
            [](ParticipantKind x)
            {
                return !x.is_valid();
            }));

    return result;
}

std::ostream& operator <<(
        std::ostream& os,
        const ParticipantKind& type)
{
    auto it = ParticipantKind::participant_type_with_aliases_.find(type.value_);

    // Value must be in map
    assert(it != ParticipantKind::participant_type_with_aliases_.end());

    os << "ParticipantKind{" << it->second[0] << "}";
    return os;
}

} /* namespace ddsrouter */
} /* namespace eprosima */
