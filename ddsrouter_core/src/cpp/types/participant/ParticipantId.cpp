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

#include <ddsrouter_core/types/participant/ParticipantId.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

//! INVALID_ID key
const std::string ParticipantId::INVALID_ID = "__invalid_ddsrouter_participant__";

ParticipantId::ParticipantId(
        const std::string& id) noexcept
    : id_(id)
{
    // If the ID cannot be used as a ParticipantId, it must store the INVALID string
    if (!ParticipantId::is_valid_id(id))
    {
        id_ = INVALID_ID;
    }
}

ParticipantId::ParticipantId() noexcept
    : id_(INVALID_ID)
{
}

ParticipantId ParticipantId::invalid() noexcept
{
    return ParticipantId();
}

bool ParticipantId::is_valid_id(
        const std::string& tag) noexcept
{
    return !tag.empty() && tag != INVALID_ID;
}

bool ParticipantId::is_valid() const noexcept
{
    return is_valid_id(id_);
}

std::string ParticipantId::id_name() const noexcept
{
    return id_;
}

bool ParticipantId::operator ==(
        const ParticipantId& other) const noexcept
{
    return id_ == other.id_;
}

bool ParticipantId::operator !=(
        const ParticipantId& other) const noexcept
{
    return !(*this == other);
}

bool ParticipantId::operator <(
        const ParticipantId& other) const noexcept
{
    return id_ < other.id_;
}

std::ostream& operator <<(
        std::ostream& os,
        const ParticipantId& id)
{
    os << "ParticipantId{" << id.id_name() << "}";
    return os;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
