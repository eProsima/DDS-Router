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
 * @file ParticipantId.hpp
 *
 * A ParticipantId is a pair of a \c ParticipantName and \c ParticipantKind. This file:
 * - Defines \c ParticipantName and declares its utility functions.
 * - Defines \c ParticipantKind and declares its utility functions.
 * - Defines \c ParticipantId and declares its utility functions.
 * - Defines \c ParticipantKeySet as a set of objects holding a ParticipantId.
 */

#ifndef _DDSROUTERCORE_TYPES_PARTICIPANTID_HPP_
#define _DDSROUTERCORE_TYPES_PARTICIPANTID_HPP_

#include <string>
#include <array>
#include <set>
#include <map>

#include <ddsrouter_core/library/library_dll.h>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

// ###################
// #                 #
// # ParticipantName #
// #                 #
// ###################

/**
 * @brief Unique Id for each DDSRouter Participant
 *
 * This class represents the ID that uniquely identifies a Participant inside the DDSRouter.
 * This unique Id is a string that behaves as the key for the unique ID.
 * Two IDs with the same string are the same ID.
 */
using ParticipantName = std::string;

static constexpr const char* InvalidParticipantName = "__invalid_ddsrouter_participant__";

DDSROUTER_CORE_DllAPI bool is_participant_name_valid(
        const ParticipantName& name);


// ###################
// #                 #
// # ParticipantKind #
// #                 #
// ###################

/**
 * @brief Underlying type alias for \c ParticipantKind enum.
 */
using ParticipantKindType = uint16_t;

/**
 * @brief Enumeration of kinds of participants.
 */
enum class ParticipantKind : ParticipantKindType
{
    invalid,                    //! Invalid Participant Kind
    blank,                      //! Blank/Void Participant Kind
    echo,                       //! Echo Participant Kind
    dummy,                      //! Dummy Participant Kind
    simple_rtps,                //! Simple RTPS Participant Kind
    local_discovery_server,     //! Discovery Server RTPS UDP Participant Kind
    wan,                        //! Discovery Server RTPS TCP Participant Kind
};

static constexpr unsigned PARTICIPANT_KIND_COUNT = 7;

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
    ParticipantKind::wan
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
    ParticipantKind::wan
};

constexpr std::array<const char*, PARTICIPANT_KIND_COUNT> PARTICIPANT_KIND_STRINGS = {
    "invalid",
    "blank",
    "echo",
    "dummy",
    "simple_rtps",
    "local_discovery_server",
    "wan"
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
    ParticipantKindAliasesType({"wan", "router", "", ""}),
};

static_assert(PARTICIPANT_KIND_ALIASES.size() == ALL_PARTICIPANT_KINDS.size());
static_assert(ALL_VALID_PARTICIPANT_KINDS.size() == ALL_PARTICIPANT_KINDS.size() - 1);

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
DDSROUTER_CORE_DllAPI ParticipantKind participant_kind_from_string(
        std::string p_kind_string);

// #################
// #               #
// # ParticipantId #
// #               #
// #################

/**
 * @brief A \c ParticipantId is a pair of a \c ParticipantName and a \c ParticipantKind.
 */
class ParticipantId : public std::pair<ParticipantName, ParticipantKind>
{
private:

    using BaseT = std::pair<ParticipantName, ParticipantKind>;

public:

    ParticipantId(
            ParticipantName name);
    ParticipantId(
            ParticipantName name,
            ParticipantKind kind);
    ParticipantId(
            ParticipantName name,
            std::string kind_str);

    DDSROUTER_CORE_DllAPI const ParticipantName& name() const noexcept;

    DDSROUTER_CORE_DllAPI ParticipantKind kind() const noexcept;
};

/**
 * @brief Return reference to ParticipantName from a ParticipantId.
 */
DDSROUTER_CORE_DllAPI const ParticipantName& get_participant_name(
        const ParticipantId& id);

/**
 * @brief Return ParticipantKind from a ParticipantId.
 */
DDSROUTER_CORE_DllAPI ParticipantKind get_participant_kind(
        const ParticipantId& id);

DDSROUTER_CORE_DllAPI bool is_participant_id_valid(
        const ParticipantId& p_id);

DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const ParticipantId& kind);

// ##############################################
// #                                            #
// # ParticipantId-aware associative containers #
// #                                            #
// ##############################################

/**
 * @brief A functor exposing comparison operators to be included in the ParticipantKeySet definition.
 *
 * This functor enables ParticipantKeySet sets to be indexed by ParticipantId, ParticipantName and const char* representations of a unique Participant.
 *
 * @tparam HoldParticipantIdPtrT
 */
template <typename HoldParticipantIdPtrT>
struct ParticipantCompare_
{
    using is_transparent = void;

    DDSROUTER_CORE_DllAPI bool operator ()(
            const HoldParticipantIdPtrT& ptr_a,
            const HoldParticipantIdPtrT& ptr_b) const;

    DDSROUTER_CORE_DllAPI bool operator ()(
            const ParticipantId& id_a,
            const HoldParticipantIdPtrT& ptr_b) const;
    DDSROUTER_CORE_DllAPI bool operator ()(
            const HoldParticipantIdPtrT& ptr_a,
            const ParticipantId& id_b) const;

    DDSROUTER_CORE_DllAPI bool operator ()(
            const ParticipantName& name_a,
            const HoldParticipantIdPtrT& id_b) const;
    DDSROUTER_CORE_DllAPI bool operator ()(
            const HoldParticipantIdPtrT& id_a,
            const ParticipantName& name_b) const;

    DDSROUTER_CORE_DllAPI bool operator ()(
            const char* name_a,
            const HoldParticipantIdPtrT& ptr_b) const;
    DDSROUTER_CORE_DllAPI bool operator ()(
            const HoldParticipantIdPtrT& ptr_a,
            const char* name_b) const;
};

/**
 * @brief Alias template type to store pointer elements encapsulating a T having an associated ParticipantId accessible via a non-static method T::id().
 *
 * Pointer elements contained in this set can be accessed by a ParticipantName index of several types, ParticipantId, ParticipantName and const char*, which uniquely identify a Participant.
 *
 * @tparam HoldParticipantIdPtrT A type having a non-static `const ParticipantId& T::id() const noexcept;` method
 */
template <typename HoldParticipantIdPtrT>
using ParticipantKeySet = std::set<HoldParticipantIdPtrT, ParticipantCompare_<HoldParticipantIdPtrT>>;

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_PARTICIPANTID_HPP_ */
