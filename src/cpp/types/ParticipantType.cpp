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
 * @file ParticipantType.cpp
 *
 */

#include <set>

#include <ddsrouter/types/ParticipantType.hpp>
#include <ddsrouter/types/utils.hpp>

namespace eprosima {
namespace ddsrouter {

ParticipantType ParticipantTypeFactory::participant_type_from_name(std::string type)
{
    // Pass type name to lowercase
    utils::to_lowercase(type);

    if (type == VOID_TYPE_NAME)
    {
        // Void type
        return ParticipantType::VOID;
    }
    else if (type == ECHO_TYPE_NAME)
    {
        // Void type
        return ParticipantType::ECHO;
    }
    else
    {
        // The type is not associated with any ParticipantType
        // TODO: Add log warning
        return ParticipantType::INVALID;
    }
}

std::ostream& operator <<(
        std::ostream& os,
        const ParticipantType& a)
{
    switch (a)
    {
    case ParticipantType::VOID:
        os << VOID_TYPE_NAME;
        break;

    case ParticipantType::ECHO:
        os << ECHO_TYPE_NAME;
        break;

    default:
        os << INVALID_TYPE_NAME_SERIALIZATION;
        break;
    }
    return os;
}

} /* namespace ddsrouter */
} /* namespace eprosima */
