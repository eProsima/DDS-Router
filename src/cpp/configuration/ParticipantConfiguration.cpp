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
#include <ddsrouter/types/participant/ParticipantType.hpp>
#include <ddsrouter/types/topic/WildcardTopic.hpp>

namespace eprosima {
namespace ddsrouter {

ParticipantConfiguration::ParticipantConfiguration(
        ParticipantId id) noexcept
    : BaseConfiguration(RawConfiguration())
    , id_(id)
    , type_(ParticipantType::PARTICIPANT_TYPE_INVALID)
{
    // Set type from id
    set_type_();
}

ParticipantConfiguration::ParticipantConfiguration(
        ParticipantId id,
        const RawConfiguration& raw_configuration)
    : BaseConfiguration(raw_configuration)
    , id_(id)
{
    if (!raw_configuration_.IsMap() && !raw_configuration_.IsNull())
    {
        throw ConfigurationException("DDSRouter Participant expects a map as base yaml type or an empty yaml");
    }

    set_type_();
    if (!type_.is_valid())
    {
        throw ConfigurationException("DDSRouter Participant expects a valid ParticipantType");
    }
}

ParticipantConfiguration::ParticipantConfiguration(
        const ParticipantConfiguration& configuration)
    : ParticipantConfiguration(configuration.id_, configuration.raw_configuration_)
{
}

void ParticipantConfiguration::set_type_() noexcept
{
    if (raw_configuration_[PARTICIPANT_TYPE_TAG])
    {
        // Get configuration type from type tag
        type_ = ParticipantType::participant_type_from_name(
            raw_configuration_[PARTICIPANT_TYPE_TAG].as<std::string>());
    }
    else
    {
        // Get configuration type from Id name
        type_ = ParticipantType::participant_type_from_name(id_.id_name());
    }
}

ParticipantType ParticipantConfiguration::type() const noexcept
{
    return type_;
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
