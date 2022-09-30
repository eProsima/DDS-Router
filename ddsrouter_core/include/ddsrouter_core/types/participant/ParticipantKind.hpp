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

#include <string>
#include <array>

#include <ddsrouter_core/library/library_dll.h>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

using ParticipantKindType = uint16_t;

enum class ParticipantKind : ParticipantKindType
{
    invalid,                    //! Invalid Participant Kind
    blank,                      //! Blank/Void Participant Kind
    echo,                       //! Echo Participant Kind
    dummy,                      //! Dummy Participant Kind
    simple_rtps,                //! Simple RTPS Participant Kind
    local_discovery_server,     //! Discovery Server RTPS Participant Kind
    wan_discovery_server,       //! Discovery Server Inter Router Participant Kind
    wan_initial_peers,          //! Initial Peers Inter Router Participant Kind
};

static constexpr unsigned PARTICIPANT_KIND_COUNT = 8;

/**
 * @brief All ParticipantKind enum values as a std::array.
 */
constexpr std::array<ParticipantKind, PARTICIPANT_KIND_COUNT> ALL_PARTICIPANT_KINDS = {
    ParticipantKind::invalid,
    ParticipantKind::blank,
    ParticipantKind::echo,
    ParticipantKind::dummy,
    ParticipantKind::simple_rtps,
    ParticipantKind::local_discovery_server,
    ParticipantKind::wan_discovery_server,
    ParticipantKind::wan_initial_peers,
};

/**
 * @brief All valid ParticipantKind enum values as a std::array.
 */
constexpr std::array<ParticipantKind, PARTICIPANT_KIND_COUNT - 1> ALL_VALID_PARTICIPANT_KINDS = {
    // ParticipantKind::invalid, // Not valid, so not included in this array
    ParticipantKind::blank,
    ParticipantKind::echo,
    ParticipantKind::dummy,
    ParticipantKind::simple_rtps,
    ParticipantKind::local_discovery_server,
    ParticipantKind::wan_discovery_server,
    ParticipantKind::wan_initial_peers,
};

constexpr std::array<const char*, PARTICIPANT_KIND_COUNT> PARTICIPANT_KIND_STRINGS = {
    "invalid",
    "blank",
    "echo",
    "dummy",
    "simple-rtps",
    "local-discovery-server",
    "wan-ds",
    "wan-initial-peers",
};

static constexpr unsigned MAX_PARTICIPANT_KIND_ALIASES = 4;

using ParticipantKindAliasesType = std::array<const char*, MAX_PARTICIPANT_KIND_ALIASES>;

/**
 * @brief All possible string aliases for each \c ParticipantKind.
 */
constexpr std::array<ParticipantKindAliasesType, PARTICIPANT_KIND_COUNT> PARTICIPANT_KIND_ALIASES = {
    ParticipantKindAliasesType({"__invalid_participant_kind__", "", "", ""}),
    ParticipantKindAliasesType({"blank", "void", "", ""}),
    ParticipantKindAliasesType({"echo", "", "", ""}),
    ParticipantKindAliasesType({"dummy", "", "", ""}),
    ParticipantKindAliasesType({"local", "simple", "", ""}),
    ParticipantKindAliasesType({"discovery-server", "ds", "local-ds", "local-discovery-server"}),
    ParticipantKindAliasesType({"wan-ds", "wan-discovery-server", "", ""}),
    ParticipantKindAliasesType({"wan", "router", "initial-peers", ""}),
};

DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        ParticipantKind kind);

/**
 * @brief Create a Participant Kind regarding the string argument in lower case
 *
 * @note Kind name is case sensitive
 *
 * It compares the argument \c kind in lower case with any of the existing kind names, and in case it
 * matches any of them, return the ParticipantKind associated with that name.
 * It will return \c ParticipantKind::invalid in case no existing ParticipantKind name matches the argument or the argument is empty.
 *
 * @param [in] kind : string with the name of the kind to build
 * @return ParticipantKind value, \c ParticipantKind::invalid if \c kind does not refer to any existing kind
 */
DDSROUTER_CORE_DllAPI ParticipantKind participant_kind_from_name(
        std::string participant_kind_str);

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_PARTICIPANTKIND_HPP_ */
