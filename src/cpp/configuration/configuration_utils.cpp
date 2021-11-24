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
#include <ddsrouter/configuration/ParticipantConfiguration.hpp>
#include <ddsrouter/types/configuration_tags.hpp>
#include <ddsrouter/types/Log.hpp>
#include <ddsrouter/exceptions/ConfigurationException.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

DomainId domain(const RawConfiguration& configuration)
{
    if (configuration[DOMAIN_ID_TAG])
    {
        // Try to get domain value from configuration
        try
        {
            DomainId domain = configuration[DOMAIN_ID_TAG].as<DomainId>();
            return  domain;
        }
        catch (const std::exception& e)
        {
            throw ConfigurationException("Domain has incorrect format");
        }
    }
    else
    {
        throw ConfigurationException("Not domain tag found");
    }
}

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */
