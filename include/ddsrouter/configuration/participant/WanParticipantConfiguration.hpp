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
 * @file WanParticipantConfiguration.hpp
 */

#ifndef _DDSROUTER_CONFIGURATION_WANPARTICIPANTCONFIGURATION_HPP_
#define _DDSROUTER_CONFIGURATION_WANPARTICIPANTCONFIGURATION_HPP_

#include <ddsrouter/configuration/participant/DiscoveryServerParticipantConfiguration.hpp>

namespace eprosima {
namespace ddsrouter {
namespace configuration {

/**
 * This class joins WAN Participant Configuration features and give methods to interact with it.
 */
class WanParticipantConfiguration : public DiscoveryServerParticipantConfiguration
{
public:

    WanParticipantConfiguration(
            const ParticipantId& id,
            const GuidPrefix& discovery_server_guid_prefix,
            const std::set<std::shared_ptr<Address>>& listening_addresses,
            const std::set<std::shared_ptr<DiscoveryServerConnectionAddress>>& connection_addresses,
            const std::map<std::string, std::string>& tls_configuration = {},
            const ParticipantType& type = ParticipantType::WAN,
            const DomainId& domain_id = DEFAULT_DS_DOMAIN_ID_);
};

} /* namespace configuration */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_CONFIGURATION_WANPARTICIPANTCONFIGURATION_HPP_ */
