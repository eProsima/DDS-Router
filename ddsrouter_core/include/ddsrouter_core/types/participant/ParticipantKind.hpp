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
 * @file ParticipantKind.hpp
 */

#ifndef _DDSROUTERCORE_TYPES_PARTICIPANTKIND_HPP_
#define _DDSROUTERCORE_TYPES_PARTICIPANTKIND_HPP_

#include <map>
#include <string>
#include <vector>

#include <ddsrouter_core/library/library_dll.h>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

using ParticipantKindType = uint16_t;

enum class ParticipantKind : ParticipantKindType {
    invalid = 0,                    //! Invalid Participant Kind
    empty = 1,                      //! Empty Participant Kind
    echo = 2,                       //! Echo Participant Kind
    dummy = 3,                      //! Dummy Participant Kind
    simple_rtps = 4,                //! Simple RTPS Participant Kind
    local_discovery_server = 5,     //! Discovery Server RTPS UDP Participant Kind
    wan = 6,                        //! Discovery Server RTPS TCP Participant Kind
};

static constexpr unsigned ParticipantKindCount = 7;

/**
 * @brief All ParticipantKind enum values as a std::array.
 */
constexpr std::array<ParticipantKind, ParticipantKindCount> AllParticipantKinds = {
    ParticipantKind::invalid,
    ParticipantKind::empty,
    ParticipantKind::echo, 
    ParticipantKind::dummy, 
    ParticipantKind::simple_rtps, 
    ParticipantKind::local_discovery_server, 
    ParticipantKind::wan
};

/**
 * @brief All valid ParticipantKind enum values as a std::array.
 */
constexpr std::array<ParticipantKind, ParticipantKindCount - 1> AllValidParticipantKinds = {
    // ParticipantKind::invalid, // Not valid, so not included in this array
    ParticipantKind::empty,
    ParticipantKind::echo, 
    ParticipantKind::dummy, 
    ParticipantKind::simple_rtps, 
    ParticipantKind::local_discovery_server, 
    ParticipantKind::wan
};

constexpr std::array<const char*, ParticipantKindCount> ParticipantKindStrings = {
    "invalid",
    "empty",
    "echo", 
    "dummy",
    "simple_rtps",
    "local_discovery_server",
    "wan",
};

static constexpr unsigned MaxParticipantKindAliases = 4;

using ParticipantKindAliasesType = std::array<const char *, MaxParticipantKindAliases>;

/**
 * @brief All possible string aliases for each \c ParticipantKind.
 */
constexpr std::array<ParticipantKindAliasesType, ParticipantKindCount> ParticipantKindAliases = {
    ParticipantKindAliasesType({"__invalid_participant_kind__", "", "", ""}),
    ParticipantKindAliasesType({"empty", "", "", ""}),
    ParticipantKindAliasesType({"echo", "", "", ""}),
    ParticipantKindAliasesType({"dummy", "", "", ""}),
    ParticipantKindAliasesType({"local", "simple", "", ""}),
    ParticipantKindAliasesType({"discovery-server", "ds", "local-ds", "local-discovery-server"}),
    ParticipantKindAliasesType({"wan", "router", "", ""}),
};

static_assert(ParticipantKindAliases.size() == AllParticipantKinds.size());
static_assert(AllValidParticipantKinds.size() == AllParticipantKinds.size() - 1);

std::ostream& operator <<(
        std::ostream& os,
        const ParticipantKind& kind);

/**
 * @brief Create a Participant Kind regarding the string argument in lower case
 *
 * @note Kind name is case sensitive
 *
 * It compares the argument \c kind in lower case with any of the existing kind names, and in case it
 * matches any of them, return the ParticipantKind associated with that name.
 * It will return \c ParticipantKind::invalid in case no existing ParticipantKind name matches the argument.
 *
 * @param [in] kind : string with the name of the kind to build
 * @return ParticipantKind value, \c ParticipantKind::invalid if \c kind does not refer to any existing kind
 */
ParticipantKind participant_kind_from_name(std::string participantKindName);

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
 
#endif /* _DDSROUTERCORE_TYPES_PARTICIPANTKIND_HPP_ */