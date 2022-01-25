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

#include <ddsrouter/configuration/ParticipantConfiguration.hpp>
#include <ddsrouter/exceptions/ConfigurationException.hpp>
#include <ddsrouter/types/configuration_tags.hpp>
#include <ddsrouter/types/participant/ParticipantKind.hpp>
#include <ddsrouter/types/topic/WildcardTopic.hpp>

namespace eprosima {
namespace ddsrouter {

ParticipantConfiguration::ParticipantConfiguration(
        ParticipantId id) noexcept
    : BaseConfiguration(RawConfiguration())
    , id_(id)
    , kind_(ParticipantKind::PARTICIPANT_KIND_INVALID)
{
    // Set kind from id
    set_kind_();
}

ParticipantConfiguration::ParticipantConfiguration(
        ParticipantId id,
        const RawConfiguration& raw_configuration)
    : BaseConfiguration(raw_configuration)
    , id_(id)
{
    if (!raw_configuration_.IsMap() && !raw_configuration_.IsNull())
    {
        throw ConfigurationException("DDSRouter Participant expects a map as base yaml or an empty yaml");
    }

    set_kind_();
    if (!kind_.is_valid())
    {
        throw ConfigurationException("DDSRouter Participant expects a valid ParticipantKind");
    }
}

ParticipantConfiguration::ParticipantConfiguration(
        const ParticipantConfiguration& configuration)
    : ParticipantConfiguration(configuration.id_, configuration.raw_configuration_)
{
}

void ParticipantConfiguration::set_kind_() noexcept
{
    if (raw_configuration_[PARTICIPANT_KIND_TAG])
    {
        // Get configuration kind from kind tag
        kind_ = ParticipantKind::participant_kind_from_name(
            raw_configuration_[PARTICIPANT_KIND_TAG].as<std::string>());
    }
    else
    {
        // Get configuration kind from Id name
        kind_ = ParticipantKind::participant_kind_from_name(id_.id_name());
    }
}

ParticipantKind ParticipantConfiguration::kind() const noexcept
{
    return kind_;
}

ParticipantId ParticipantConfiguration::id() const noexcept
{
    return id_;
}

RawConfiguration ParticipantConfiguration::raw_configuration() const noexcept
{
    return raw_configuration_;
}

bool ParticipantConfiguration::operator ==(
        const ParticipantConfiguration& other) const noexcept
{
    return this->id_ == other.id_ && this->raw_configuration_ == other.raw_configuration_;
}

} /* namespace ddsrouter */
} /* namespace eprosima */
