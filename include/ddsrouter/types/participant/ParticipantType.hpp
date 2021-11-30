
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
 * @file ParticipantType.hpp
 */

#ifndef _DDSROUTER_TYPES_PARTICIPANTTYPE_HPP_
#define _DDSROUTER_TYPES_PARTICIPANTTYPE_HPP_

#include <map>
#include <string>

namespace eprosima {
namespace ddsrouter {

using ParticipantTypeValue = uint16_t;
/**
 * @brief Object that handles the creation of ParticipantTypes from a string
 */
class ParticipantType
{
public:

    /**
     * @brief Participant types available
     *
     * This list must be extended for each new Participant that is implemented
     */
    enum
    {
        PARTICIPANT_TYPE_INVALID,    //! Invalid Participant Type
        VOID,       //! Void Participant Type
        ECHO,       //! Echo Participant Type
        DUMMY,      //! Dummy Participant Type
        SIMPLE_RTPS,      //! Simple RTPS Participant Type
    };

    //! Default constructor that returns an Invalid Participant Type
    ParticipantType() noexcept;

    //! Constructor by value of the ParticipantType enum
    ParticipantType(
            ParticipantTypeValue value) noexcept;

    /**
     * @brief Whether this ParticipantType is valid
     *
     * @return true if valid. False otherwise
     */
    bool is_valid() const noexcept;

    //! Convert this ParticipantType to string using the << operator
    std::string to_string() const noexcept;

    //! Return the enum value of this object
    ParticipantTypeValue operator ()() const noexcept;

    /**
     * @brief Create a Participant Type regarding the string argument in lower case
     *
     * @note Type name is not case sensitive
     *
     * It compares the argument \c type in lower case with any of the existing type names, and in case it
     * matches any of them, return the ParticipantType associated with that name.
     * It will return \c INVALID in case no existing ParticipantType name matches the argument.
     *
     * @param [in] type : string with the name of the type to build
     * @return ParticipantType value, \c INVALID if \c type does not refer to any existing type
     */
    static ParticipantType participant_type_from_name(
            std::string type) noexcept;

protected:

    //! Value that links with the enumeration ParticipantType of this object
    ParticipantTypeValue value_;

    //! Map that link a \c ParticipantType with its name to be created or deserialized
    static const std::map<ParticipantTypeValue, std::string> participant_type_with_name_;

    // Allow operator << to use \c participant_type_with_name_
    friend std::ostream& operator <<(
            std::ostream& os,
            const ParticipantType& type);
};

//! \c ParticipantType to stream serializator
std::ostream& operator <<(
        std::ostream& os,
        const ParticipantType& a);

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TYPES_PARTICIPANTTYPE_HPP_ */
