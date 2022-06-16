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
 * @file ParticipantId.cpp
 *
 */

#include <set>
#include <sstream>
#include <algorithm>
#include <assert.h>
#include <set>
#include <iostream>

#include <ddsrouter_utils/utils.hpp>
#include <ddsrouter_utils/exception/InitializationException.hpp>
#include <ddsrouter_core/types/participant/ParticipantId.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

// ###################
// #                 #
// # ParticipantName #
// #                 #
// ###################

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

bool is_participant_name_valid(
        const ParticipantName& name)
{
    return !name.empty() && name != std::string(InvalidParticipantName);
}

// ###################
// #                 #
// # ParticipantKind #
// #                 #
// ###################

ParticipantKind participant_kind_from_string(
        std::string kind_str)
{

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

// #################
// #               #
// # ParticipantId #
// #               #
// #################

ParticipantId::ParticipantId(
        ParticipantName name)
    : BaseT(std::make_pair(name, ParticipantKind::blank))
{

    if (not is_participant_id_valid(*this))
    {
        throw utils::InitializationException(utils::Formatter() << "Invalid participant ID" << *this);
    }
}

ParticipantId::ParticipantId(
        ParticipantName name,
        ParticipantKind kind)
    : BaseT(std::make_pair(name, kind))
{

    if (not is_participant_id_valid(*this))
    {
        throw utils::InitializationException(utils::Formatter() << "Invalid participant ID" << *this);
    }
}

ParticipantId::ParticipantId(
        ParticipantName name,
        std::string kind_str)
    : BaseT(std::make_pair(name, participant_kind_from_string(kind_str)))
{

    if (not is_participant_id_valid(*this))
    {
        throw utils::InitializationException(utils::Formatter() << "Invalid participant ID" << *this);
    }
}

const ParticipantName& ParticipantId::name() const noexcept
{
    return std::get<ParticipantName>(static_cast<const BaseT&>(*this));
}

ParticipantKind ParticipantId::kind() const noexcept
{
    return std::get<ParticipantKind>(static_cast<const BaseT&>(*this));
}

/**
 * @brief Return reference to ParticipantName from a ParticipantId.
 */
const ParticipantName& get_participant_name(
        const ParticipantId& id)
{
    return std::get<ParticipantName>(id);
}

/**
 * @brief Return ParticipantKind from a ParticipantId.
 */
ParticipantKind get_participant_kind(
        const ParticipantId& id)
{
    return std::get<ParticipantKind>(id);
}

bool is_participant_id_valid(
        const ParticipantId& p_id)
{
    return is_participant_name_valid(get_participant_name(p_id)) &&
           get_participant_kind(p_id) != ParticipantKind::invalid;
}

std::ostream& operator <<(
        std::ostream& os,
        const ParticipantId& id)
{
    os << "{" << id.name() << ";" << id.kind() << "}";
    return os;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
