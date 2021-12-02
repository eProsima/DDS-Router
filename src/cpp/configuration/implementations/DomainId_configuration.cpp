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
 * @file DomainId_configuration.cpp
 *
 */

#include <ddsrouter/exceptions/ConfigurationException.hpp>
#include <ddsrouter/exceptions/UnsupportedException.hpp>
#include <ddsrouter/types/configuration_tags.hpp>
#include <ddsrouter/types/endpoint/DomainId.hpp>
#include <ddsrouter/types/RawConfiguration.hpp>

namespace eprosima {
namespace ddsrouter {

DomainId::DomainId(
    const RawConfiguration& configuration)
{
    if (configuration[DOMAIN_ID_TAG])
    {
        // Try to get domain value from configuration
        try
        {
            domain_id_ = configuration[DOMAIN_ID_TAG].as<DomainIdType>();
        }
        catch (const std::exception& e)
        {
            throw ConfigurationException(utils::Formatter() <<
                "Domain has incorrect format" << e.what());
        }
    }
    else
    {
        throw ConfigurationException(
            "Not domain tag found");
    }
}

RawConfiguration DomainId::dump(RawConfiguration&) const
{
    // TODO
    throw UnsupportedException("DomainId::dump is not supported yet.");
}

} /* namespace ddsrouter */
} /* namespace eprosima */
