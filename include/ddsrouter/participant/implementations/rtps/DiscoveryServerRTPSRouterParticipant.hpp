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
 * @file DiscoveryServerRTPSRouterParticipant.hpp
 */

#ifndef _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_AUX_DISCOVERYSERVERRTPSROUTERPARTICIPANT_HPP_
#define _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_AUX_DISCOVERYSERVERRTPSROUTERPARTICIPANT_HPP_

#include <ddsrouter/configuration/DiscoveryServerRTPSParticipantConfiguration.hpp>
#include <ddsrouter/participant/implementations/rtps/CommonRTPSRouterParticipant.hpp>

namespace eprosima {
namespace ddsrouter {
namespace rtps {

/**
 * TODO
 */
template <class ConfigurationType>
class DiscoveryServerRTPSRouterParticipant
    : public CommonRTPSRouterParticipant<ConfigurationType>
{
public:

// Force ConfigurationType to be subclass of DiscoveryServerRTPSParticipantConfiguration
FORCE_TEMPLATE_SUBCLASS(DiscoveryServerRTPSParticipantConfiguration, ConfigurationType);

    DiscoveryServerRTPSRouterParticipant(
            const ParticipantConfiguration& participant_configuration,
            std::shared_ptr<PayloadPool> payload_pool,
            std::shared_ptr<DiscoveryDatabase> discovery_database);

    virtual fastrtps::rtps::RTPSParticipantAttributes participant_attributes_() const noexcept override;
};

} /* namespace rpts */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter/participant/implementations/rtps/impl/DiscoveryServerRTPSRouterParticipant.ipp>

#endif /* _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_AUX_DISCOVERYSERVERRTPSROUTERPARTICIPANT_HPP_ */