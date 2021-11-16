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
 *
 */

#include <ddsrouter/configuration/ParticipantConfiguration.hpp>
#include <ddsrouter/types/configuration_tags.hpp>
#include <ddsrouter/types/topic/WildcardTopic.hpp>
#include <ddsrouter/exceptions/ConfigurationException.hpp>
#include <ddsrouter/types/ParticipantType.hpp>

namespace eprosima {
namespace ddsrouter {

// TODO: Add logs

ParticipantConfiguration::ParticipantConfiguration(
            ParticipantId id,
            const RawConfiguration& raw_configuration)
    : id_(id)
    , raw_configuration_(raw_configuration)
{
    if (!raw_configuration_.IsMap() && !raw_configuration_.IsNull())
    {
        throw ConfigurationException("DDSRouter Participant expects a map as base yaml type or an empty");
    }

    set_type_();
    if (type_ == ParticipantType::INVALID)
    {
        throw ConfigurationException("DDSRouter Participant expects a valid ParticipantType");
    }
}

ParticipantConfiguration::~ParticipantConfiguration()
{
}

void ParticipantConfiguration::set_type_()
{
    if (raw_configuration_[PARTICIPANT_TYPE_TAG])
    {
        // Get configuration type from type tag
        type_ = ParticipantTypeFactory::participant_type_from_name(
            raw_configuration_[PARTICIPANT_TYPE_TAG].as<std::string>());
    }
    else
    {
        // Get configuration type from Id name
        type_ = ParticipantTypeFactory::participant_type_from_name(id_.id_name());
    }
}


ParticipantType ParticipantConfiguration::type() const
{
    return type_;
}

ParticipantId ParticipantConfiguration::id() const
{
    return id_;
}

ParticipantConfiguration& ParticipantConfiguration::operator =(
        const ParticipantConfiguration& other)
{
    this->id_ = other.id_;
    this->type_ = other.type_;
    this->raw_configuration_ = other.raw_configuration_;
    return *this;
}

bool ParticipantConfiguration::operator ==(
        const ParticipantConfiguration& other) const
{
    return this->id_ == other.id_ && this->raw_configuration_ == other.raw_configuration_;
}

} /* namespace ddsrouter */
} /* namespace eprosima */
