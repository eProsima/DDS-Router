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
 * @file DiscoveryServerRTPSParticipantConfiguration.hpp
 */

#ifndef _DDSROUTER_CONFIGURATION_DISCOVERYSERVERRTPSPARTICIPANTCONFIGURATION_HPP_
#define _DDSROUTER_CONFIGURATION_DISCOVERYSERVERRTPSPARTICIPANTCONFIGURATION_HPP_

#include <ddsrouter/configuration/ParticipantConfiguration.hpp>
#include <ddsrouter/types/dds_types.hpp>
#include <ddsrouter/types/address/Address.hpp>
#include <ddsrouter/types/address/DiscoveryServerAddress.hpp>
#include <ddsrouter/configuration/configuration_utils.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * This class joins every DDSRouter Participant Configuration characteristic and give methods to interact with it.
 * Each Participant that require specific configuration must inherit from this class.
 */
class DiscoveryServerRTPSParticipantConfiguration : public ParticipantConfiguration
{
public:

    // Using parent constructors
    using ParticipantConfiguration::ParticipantConfiguration;

    DiscoveryServerRTPSParticipantConfiguration(
            const ParticipantConfiguration& configuration);

    std::vector<Address> listening_addresses() const noexcept;

    std::vector<DiscoveryServerAddress> connection_addresses() const noexcept;

    GuidPrefix discovery_server_guid() const noexcept;

    DomainId domain() const noexcept;

protected:

    static const DomainId DEFAULT_DS_DOMAIN_ID_; // 66
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_CONFIGURATION_DISCOVERYSERVERRTPSPARTICIPANTCONFIGURATION_HPP_ */
