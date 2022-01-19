
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

#ifndef _DDSROUTER_TYPES_PARTICIPANTTYPE_HPP_
#define _DDSROUTER_TYPES_PARTICIPANTTYPE_HPP_

#include <map>
#include <string>
#include <vector>

namespace eprosima {
namespace ddsrouter {

using ParticipantKindValue = uint16_t;
/**
 * @brief Object that handles the creation of ParticipantKinds from a string
 */
class ParticipantKind
{
public:

    /**
     * @brief Participant types available
     *
     * This list must be extended for each new Participant that is implemented
     */
    enum
    {
        PARTICIPANT_TYPE_INVALID,   //! Invalid Participant Type
        VOID,                       //! Void Participant Type
        ECHO,                       //! Echo Participant Type
        DUMMY,                      //! Dummy Participant Type
        SIMPLE_RTPS,                //! Simple RTPS Participant Type
        LOCAL_DISCOVERY_SERVER,     //! Discovery Server RTPS UDP Participant Type
        WAN,                        //! Discovery Server RTPS TCP Participant Type
    };

    //! Default constructor that returns an Invalid Participant Type
    ParticipantKind() noexcept;

    //! Constructor by value of the ParticipantKind enum
    ParticipantKind(
            ParticipantKindValue value) noexcept;

    /**
     * @brief Whether this ParticipantKind is valid
     *
     * @return true if valid. False otherwise
     */
    bool is_valid() const noexcept;

    //! Convert this ParticipantKind to string using the << operator
    std::string to_string() const noexcept;

    //! Return the enum value of this object
    ParticipantKindValue operator ()() const noexcept;

    //! Minor operator
    bool operator <(
            const ParticipantKind& other) const noexcept;

    //! Equal operator
    bool operator ==(
            const ParticipantKind& other) const noexcept;

    /**
     * @brief Create a Participant Type regarding the string argument in lower case
     *
     * @note Type name is not case sensitive
     *
     * It compares the argument \c type in lower case with any of the existing type names, and in case it
     * matches any of them, return the ParticipantKind associated with that name.
     * It will return \c INVALID in case no existing ParticipantKind name matches the argument.
     *
     * @param [in] type : string with the name of the type to build
     * @return ParticipantKind value, \c INVALID if \c type does not refer to any existing type
     */
    static ParticipantKind participant_type_from_name(
            std::string type) noexcept;

    static std::vector<ParticipantKind> all_valid_participant_types() noexcept;

protected:

    //! Value that links with the enumeration ParticipantKind of this object
    ParticipantKindValue value_;

    //! Map that link a \c ParticipantKind with its name to be created or deserialized
    static const std::map<ParticipantKindValue, std::vector<std::string>> participant_type_with_aliases_;

    // Allow operator << to use \c participant_type_with_aliases_
    friend std::ostream& operator <<(
            std::ostream& os,
            const ParticipantKind& type);
};

//! \c ParticipantKind to stream serializator
std::ostream& operator <<(
        std::ostream& os,
        const ParticipantKind& a);

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TYPES_PARTICIPANTTYPE_HPP_ */
