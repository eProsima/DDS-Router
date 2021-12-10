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
 * @file WANParticipantConfiguration.hpp
 */

#ifndef _DDSROUTER_CONFIGURATION_WANPARTICIPANTCONFIGURATION_HPP_
#define _DDSROUTER_CONFIGURATION_WANPARTICIPANTCONFIGURATION_HPP_

#include <ddsrouter/configuration/DiscoveryServerParticipantConfiguration.hpp>
#include <ddsrouter/configuration/ParticipantConfiguration.hpp>
#include <ddsrouter/types/address/Address.hpp>
#include <ddsrouter/types/address/DiscoveryServerConnectionAddress.hpp>
#include <ddsrouter/types/endpoint/DomainId.hpp>
#include <ddsrouter/types/endpoint/GuidPrefix.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * This class joins WAN Participant Configuration features and give methods to interact with it.
 */
class WANParticipantConfiguration : public DiscoveryServerParticipantConfiguration
{
public:

    // Using parent constructors
    using DiscoveryServerParticipantConfiguration::DiscoveryServerParticipantConfiguration;

    WANParticipantConfiguration(
            const ParticipantConfiguration& configuration);

protected:

    TransportProtocol default_transport_protocol_() const noexcept override;

    static const TransportProtocol DEFAULT_TRANSPORT_PROTOCOL_; // UDP
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_CONFIGURATION_WANPARTICIPANTCONFIGURATION_HPP_ */
