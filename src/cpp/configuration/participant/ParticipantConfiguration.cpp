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
 * @file ParticipantConfiguration.cpp
 */

#include <ddsrouter/configuration/participant/ParticipantConfiguration.hpp>
#include <ddsrouter/exceptions/ConfigurationException.hpp>
#include <ddsrouter/types/participant/ParticipantKind.hpp>
#include <ddsrouter/types/topic/WildcardTopic.hpp>

namespace eprosima {
namespace ddsrouter {
namespace configuration {

ParticipantConfiguration::ParticipantConfiguration(
        const ParticipantId& id,
        const ParticipantKind& kind) noexcept
    : id_(id)
    , kind_(kind)
{
}

ParticipantKind ParticipantConfiguration::kind() const noexcept
{
    return kind_;
}

ParticipantId ParticipantConfiguration::id() const noexcept
{
    return id_;
}

bool ParticipantConfiguration::is_valid(
        utils::Formatter& error_msg) const noexcept
{
    if (!id_.is_valid())
    {
        error_msg << "Non valid Participant Id " << id_ << ". ";
        return false;
    }

    if (!kind_.is_valid())
    {
        error_msg << "Non valid Participant kind " << kind_ << ". ";
        return false;
    }

    return true;
}

bool ParticipantConfiguration::operator ==(
        const ParticipantConfiguration& other) const noexcept
{
    return this->id() == other.id() && this->kind() == other.kind();
}

} /* namespace configuration */
} /* namespace ddsrouter */
} /* namespace eprosima */
