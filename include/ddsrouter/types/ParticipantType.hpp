
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

//! Participant types available
enum ParticipantType
{
    INVALID,    //! Invalid Participant Type
    VOID,       //! Void Participant Type
};

/**
 * @brief Object that handles the creation of ParticipantTypes from a string
 */
class ParticipantTypeFactory
{
public:

    /**
     * @brief Create a Participant Type regarding the string argument in lower case
     *
     * @note Type name is not case sensitive
     *
     * It compares the argument \c type in lower case with any of the existing type names, and in case it
     * matches any of them, return the ParticipantType associated with that name.
     * It will return \c INVALID in case no existing ParticipantType name matches the argument.
     *
     * @param [out] type : string with the name of the type to build
     * @return ParticipantType value, \c INVALID if \c type does not refer to any existing type
     */
    static ParticipantType participant_type_from_name(std::string type);

protected:

    //! String on how the Invalid Partiticipant Type will be serialized
};

constexpr const char* VOID_TYPE_NAME("void");   //! Void participant type name

constexpr const char* INVALID_TYPE_NAME_SERIALIZATION("InvalidParticipantType"); //! Serialization of invalid type

//! \c ParticipantType to stream serializator
std::ostream& operator <<(
        std::ostream& os,
        const ParticipantType& a);

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TYPES_PARTICIPANTTYPE_HPP_ */
