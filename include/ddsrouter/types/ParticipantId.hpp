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
 */

#ifndef _DDSROUTER_TYPES_PARTICIPANTID_HPP_
#define _DDSROUTER_TYPES_PARTICIPANTID_HPP_

#include <string>

namespace eprosima {
namespace ddsrouter {

//! Unique Id for each DDSRouter Participant
class ParticipantId
{
public:

    ParticipantId();

    ParticipantId(
            const std::string& id);

    virtual ~ParticipantId();

    static ParticipantId invalid();

    static bool is_valid_id(
            const std::string& id);

    bool is_valid() const;

    std::string id_name() const;

    // OPERATOR OVERLOAD
    bool operator ==(
            const ParticipantId& other) const;

    bool operator <(
            const ParticipantId& other) const;

protected:

    static const std::string INVALID_ID; // __invalid_ddsrouter_participant__

    std::string id_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TYPES_PARTICIPANTID_HPP_ */
