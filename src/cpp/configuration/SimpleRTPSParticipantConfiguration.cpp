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

#include <ddsrouter/configuration/SimpleRTPSParticipantConfiguration.hpp>
#include <ddsrouter/configuration/configuration_utils.hpp>
#include <ddsrouter/types/Log.hpp>

namespace eprosima {
namespace ddsrouter {

SimpleRTPSParticipantConfiguration::SimpleRTPSParticipantConfiguration(
        const ParticipantConfiguration& configuration)
    : ParticipantConfiguration(configuration.id(), configuration.raw_configuration())
{
}

DomainId SimpleRTPSParticipantConfiguration::domain() const noexcept
{
    // TODO
    try
    {
        return utils::domain(raw_configuration_);
    }
    catch(const std::exception& e)
    {
        logInfo(DDSROUTER_SIMPLE_RTPS_CONFIGURATION,
            "Error getting domain in Participant " << id() << " with error: " << e.what());

        return utils::DEFAULT_DOMAIN_ID;
    }
}

} /* namespace ddsrouter */
} /* namespace eprosima */
