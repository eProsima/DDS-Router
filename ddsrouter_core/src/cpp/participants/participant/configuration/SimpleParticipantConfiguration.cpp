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
 * @file SimpleParticipantConfiguration.cpp
 */

#include <cpp_utils/Log.hpp>

#include <ddsrouter_core/participants/participant/configuration/SimpleParticipantConfiguration.hpp>

namespace eprosima {
namespace ddsrouter {
namespace participants {

using namespace eprosima::ddsrouter::core::types;

SimpleParticipantConfiguration::SimpleParticipantConfiguration(
        const ParticipantId& id,
        const bool is_repeater,
        const DomainId& domain_id) noexcept
    : ParticipantConfiguration(id, is_repeater)
    , domain(domain_id)
{
}

bool SimpleParticipantConfiguration::is_valid(
        utils::Formatter& error_msg) const noexcept
{
    if (!ParticipantConfiguration::is_valid(error_msg))
    {
        return false;
    }

    if (!domain.is_valid())
    {
        error_msg << "Incorrect domain " << domain << ". ";
        return false;
    }

    return true;
}

bool SimpleParticipantConfiguration::operator ==(
        const SimpleParticipantConfiguration& other) const noexcept
{
    return ParticipantConfiguration::operator ==(
        other) &&
           this->domain == other.domain;
}

} /* namespace participants */
} /* namespace ddsrouter */
} /* namespace eprosima */
