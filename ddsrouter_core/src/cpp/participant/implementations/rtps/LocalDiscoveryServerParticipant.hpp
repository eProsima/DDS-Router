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
 * @file LocalDiscoveryServerParticipant.hpp
 */

#ifndef __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_RTPS_LOCALDISCOVERYSERVERPARTICIPANT_HPP_
#define __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_RTPS_LOCALDISCOVERYSERVERPARTICIPANT_HPP_

#include <ddsrouter_core/configuration/participant/DiscoveryServerParticipantConfiguration.hpp>
#include <participant/implementations/rtps/DiscoveryServerParticipant.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

/**
 * TODO
 */
class LocalDiscoveryServerParticipant
    : public DiscoveryServerParticipant<configuration::DiscoveryServerParticipantConfiguration>
{
public:

    LocalDiscoveryServerParticipant(
            const configuration::DiscoveryServerParticipantConfiguration participant_configuration,
            std::shared_ptr<PayloadPool> payload_pool,
            std::shared_ptr<fastrtps::rtps::IChangePool> cache_change_pool,
            std::shared_ptr<DiscoveryDatabase> discovery_database);
};

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_RTPS_LOCALDISCOVERYSERVERPARTICIPANT_HPP_ */
