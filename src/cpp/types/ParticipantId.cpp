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

#include <ddsrouter/types/ParticipantId.hpp>
#include <ddsrouter/types/configuration_tags.hpp>

namespace eprosima {
namespace ddsrouter {

const std::string ParticipantId::INVALID_ID = "__invalid_ddsrouter_participant__";

ParticipantId::ParticipantId(
        const std::string& id)
    : id_(id)
{
}

ParticipantId::ParticipantId()
    : id_(INVALID_ID)
{
}

ParticipantId::~ParticipantId()
{
}

ParticipantId ParticipantId::invalid()
{
    return ParticipantId();
}

bool ParticipantId::is_valid_id(
        const std::string& tag)
{
    return (tag != WHITELIST_TAG) && (tag != BLOCKLIST_TAG);
}

bool ParticipantId::is_valid() const
{
    return id_ != INVALID_ID;
}

bool ParticipantId::operator ==(
        const ParticipantId& other) const
{
    return id_ == other.id_;
}

bool ParticipantId::operator <(
        const ParticipantId& other) const
{
    return id_ < other.id_;
}

} /* namespace ddsrouter */
} /* namespace eprosima */
