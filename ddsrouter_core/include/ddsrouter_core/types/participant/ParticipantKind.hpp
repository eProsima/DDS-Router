
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

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

using ParticipantKindType = uint16_t;
/**
 * @brief Object that handles the creation of ParticipantKinds from a string
 */
class ParticipantKind
{
public:

    /**
     * @brief Participant kinds available
     *
     * This list must be extended for each new Participant that is implemented
     */
    enum
    {
        PARTICIPANT_KIND_INVALID,   //! Invalid Participant Kind
        VOID,                       //! Void Participant Kind
        ECHO,                       //! Echo Participant Kind
        DUMMY,                      //! Dummy Participant Kind
        SIMPLE_RTPS,                //! Simple RTPS Participant Kind
        LOCAL_DISCOVERY_SERVER,     //! Discovery Server RTPS UDP Participant Kind
        WAN,                        //! Discovery Server RTPS TCP Participant Kind
    };

    //! Default constructor that returns an Invalid Participant Kind
    ParticipantKind() noexcept;

    //! Constructor by value of the ParticipantKind enum
    ParticipantKind(
            ParticipantKindType value) noexcept;

    /**
     * @brief Whether this ParticipantKind is valid
     *
     * @return true if valid. False otherwise
     */
    bool is_valid() const noexcept;

    //! Convert this ParticipantKind to string using the << operator
    std::string to_string() const noexcept;

    //! Return the enum value of this object
    ParticipantKindType operator ()() const noexcept;

    //! Minor operator
    bool operator <(
            const ParticipantKind& other) const noexcept;

    //! Equal operator
    bool operator ==(
            const ParticipantKind& other) const noexcept;

    /**
     * @brief Create a Participant Kind regarding the string argument in lower case
     *
     * @note Kind name is not case sensitive
     *
     * It compares the argument \c kind in lower case with any of the existing kind names, and in case it
     * matches any of them, return the ParticipantKind associated with that name.
     * It will return \c INVALID in case no existing ParticipantKind name matches the argument.
     *
     * @param [in] kind : string with the name of the kind to build
     * @return ParticipantKind value, \c INVALID if \c kind does not refer to any existing kind
     */
    static ParticipantKind participant_kind_from_name(
            std::string kind) noexcept;

    static std::vector<ParticipantKind> all_valid_participant_kinds() noexcept;

protected:

    //! Value that links with the enumeration ParticipantKind of this object
    ParticipantKindType value_;

    //! Map that link a \c ParticipantKind with its name to be created or deserialized
    static const std::map<ParticipantKindType, std::vector<std::string>> participant_kind_with_aliases_;

    // Allow operator << to use \c participant_kind_with_aliases_
    friend std::ostream& operator <<(
            std::ostream& os,
            const ParticipantKind& kind);
};

//! \c ParticipantKind to stream serializator
std::ostream& operator <<(
        std::ostream& os,
        const ParticipantKind& a);

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_PARTICIPANTKIND_HPP_ */
