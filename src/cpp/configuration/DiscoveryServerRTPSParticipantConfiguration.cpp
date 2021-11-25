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

#include <ddsrouter/configuration/DiscoveryServerRTPSParticipantConfiguration.hpp>
#include <ddsrouter/configuration/configuration_utils.hpp>
#include <ddsrouter/types/Log.hpp>

namespace eprosima {
namespace ddsrouter {

const DomainId DiscoveryServerRTPSParticipantConfiguration::DEFAULT_DS_DOMAIN_ID_(66);

DiscoveryServerRTPSParticipantConfiguration::DiscoveryServerRTPSParticipantConfiguration(
        const ParticipantConfiguration& configuration)
    : ParticipantConfiguration(configuration.id(), configuration.raw_configuration())
{
}

std::vector<Address> DiscoveryServerRTPSParticipantConfiguration::listening_addresses() const noexcept
{
    std::vector<Address> result;
    // TODO

    return result;
}

std::vector<DiscoveryServerAddress> DiscoveryServerRTPSParticipantConfiguration::connection_addresses() const noexcept
{
    std::vector<DiscoveryServerAddress> result;
    // TODO

    return result;
}

GuidPrefix DiscoveryServerRTPSParticipantConfiguration::discovery_server_guid() const noexcept
{
    // TODO
    return GuidPrefix();
}

DomainId DiscoveryServerRTPSParticipantConfiguration::domain() const noexcept
{
    // This method always return the same domain for the Discovery Server in order
    // to avoid problems with the LogicalPort
    return DEFAULT_DS_DOMAIN_ID_;
}

} /* namespace ddsrouter */
} /* namespace eprosima */
