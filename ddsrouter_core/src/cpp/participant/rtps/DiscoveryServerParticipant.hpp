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
 * @file DiscoveryServerParticipant.hpp
 */

#ifndef __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_RTPS_DISCOVERYSERVERPARTICIPANT_HPP_
#define __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_RTPS_DISCOVERYSERVERPARTICIPANT_HPP_

#include <fastdds/rtps/transport/TCPTransportDescriptor.h>

#include <ddsrouter_core/configuration/participant/DiscoveryServerParticipantConfiguration.hpp>
#include <ddsrouter_core/types/security/tls/TlsConfiguration.hpp>

#include <participant/rtps/CommonRTPSRouterParticipant.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

/**
 * Intermediate class serving as a base class for ParticipantKind::local_discovery_server and ParticipantKind::wan
 */
class DiscoveryServerParticipant
    : public CommonRTPSRouterParticipant
{
public:

    using CommonRTPSRouterParticipant::CommonRTPSRouterParticipant;

    DiscoveryServerParticipant(
            const configuration::ParticipantConfiguration& participant_configuration,
            DiscoveryDatabase& discovery_database);

    virtual fastrtps::rtps::RTPSParticipantAttributes participant_attributes_() const override;

    static void enable_tls(
            std::shared_ptr<eprosima::fastdds::rtps::TCPTransportDescriptor> descriptor,
            const types::security::TlsConfiguration& tls_configuration,
            bool client = false);

    static void enable_tls_client(
            std::shared_ptr<eprosima::fastdds::rtps::TCPTransportDescriptor> descriptor,
            const types::security::TlsConfiguration& tls_configuration,
            bool only_client);

    static void enable_tls_server(
            std::shared_ptr<eprosima::fastdds::rtps::TCPTransportDescriptor> descriptor,
            const types::security::TlsConfiguration& tls_configuration);
};

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_RTPS_DISCOVERYSERVERPARTICIPANT_HPP_ */
