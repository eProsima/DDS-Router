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
 * @file ShmParticipant.cpp
 */

#include <memory>

#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.h>

#include <participant/implementations/rtps/ShmParticipant.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

using namespace eprosima::ddsrouter::core::types;

ShmParticipant::ShmParticipant(
        const configuration::SimpleParticipantConfiguration participant_configuration,
        std::shared_ptr<PayloadPool> payload_pool,
        std::shared_ptr<DiscoveryDatabase> discovery_database)
    : CommonRTPSRouterParticipant<configuration::SimpleParticipantConfiguration>(
            participant_configuration, payload_pool, discovery_database)
{
    create_participant_();
}

fastrtps::rtps::RTPSParticipantAttributes ShmParticipant::participant_attributes_() const
{
    fastrtps::rtps::RTPSParticipantAttributes att = CommonRTPSRouterParticipant::participant_attributes_();

    // Do not use default builtin
    att.useBuiltinTransports = false;

    // Add SHM transport
    auto shm_transport = std::make_shared<eprosima::fastdds::rtps::SharedMemTransportDescriptor>();
    att.userTransports.push_back(shm_transport);

    return att;
}

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
