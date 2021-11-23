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
 * @file SimpleRTPSParticipantConfiguration.cpp
 */

#include <ddsrouter/configuration/SimpleRTPSParticipantConfiguration.hpp>
#include <ddsrouter/types/configuration_tags.hpp>
#include <ddsrouter/types/Log.hpp>

namespace eprosima {
namespace ddsrouter {

DomainId SimpleRTPSParticipantConfiguration::domain() const noexcept
{
    DomainId domain = 0; // Default 0

    if (raw_configuration_[DOMAIN_ID_TAG])
    {
        // Try to get domain value from configuration
        try
        {
            domain = raw_configuration_[DOMAIN_ID_TAG].as<DomainId>();
        }
        catch (const std::exception& e)
        {
            // Using default domain
            logWarning(SIMPLE_RTPS_PARTICIPANT_CONFIGURATION,
                "Incorrect Domain Configuration " << raw_configuration_[DOMAIN_ID_TAG] << " " <<
                "while reading configuration for Participant: " << id() << ". " <<
                "Domain must be an unsigned integer. Using default DomainId 0.");

        }
    }
    else
    {
        // If it is not specified use domain 0
        logInfo(SIMPLE_RTPS_PARTICIPANT_CONFIGURATION,
            "Participant " << id() << " has not domain specified. Using default DomainId 0.");
    }

    return domain;
}

} /* namespace ddsrouter */
} /* namespace eprosima */
