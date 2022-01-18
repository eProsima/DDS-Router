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
 * @file SimpleParticipant.hpp
 */

#ifndef _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_RTPS_SIMPLEPARTICIPANT_HPP_
#define _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_RTPS_SIMPLEPARTICIPANT_HPP_

#include <ddsrouter/configuration/participant/SimpleParticipantConfiguration.hpp>
#include <ddsrouter/participant/implementations/rtps/CommonRTPSRouterParticipant.hpp>

namespace eprosima {
namespace ddsrouter {
namespace rtps {

/**
 * Participant with Simple Discovery Protocol.
 *
 * Standard RTPS Participant with Simple Discovery and default attributes.
 */
class SimpleParticipant : public CommonRTPSRouterParticipant<configuration::SimpleParticipantConfiguration>
{
public:

    /**
     * @brief Construct a new Dummy Participant object
     *
     * It uses the \c BaseParticipant constructor.
     * Apart from BaseParticipant, it creates a new RTPSParticipant with default Attributes and domain given
     * by configuration.
     *
     * @throw \c InitializationException in case any internal error has ocurred while creating RTPSParticipant
     * @throw \c IConfigurationException in case configuration was incorrectly set
     */
    SimpleParticipant(
            std::shared_ptr<configuration::SimpleParticipantConfiguration> participant_configuration,
            std::shared_ptr<PayloadPool> payload_pool,
            std::shared_ptr<DiscoveryDatabase> discovery_database);
};

} /* namespace rtps */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_RTPS_SIMPLEPARTICIPANT_HPP_ */
