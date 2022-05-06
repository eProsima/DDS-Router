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
 * @file EchoDiscoveryParticipant.cpp
 */

#include <participant/implementations/auxiliar/EchoDiscoveryParticipant.hpp>
#include <reader/implementations/auxiliar/VoidReader.hpp>
#include <writer/implementations/auxiliar/VoidWriter.hpp>
#include <ddsrouter_core/types/participant/ParticipantKind.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

using namespace eprosima::ddsrouter::core::types;

EchoDiscoveryParticipant::EchoDiscoveryParticipant(
        const configuration::ParticipantConfiguration participant_configuration,
        std::shared_ptr<PayloadPool> payload_pool,
        std::shared_ptr<DiscoveryDatabase> discovery_database)
    : BaseParticipant<configuration::ParticipantConfiguration>(
        participant_configuration, payload_pool, discovery_database)
{
    // Register in Discovery DB a callback to be notified each time an endpoint is discovered
    discovery_database_->add_endpoint_discovered_callback(
        [this](const Endpoint& endpoint_discovered) {
            this->echo_discovery(endpoint_discovered);
        });
}

void EchoDiscoveryParticipant::echo_discovery(Endpoint endpoint_discovered) const noexcept
{
    // TODO write this in a way that is efficient and easy to read
    logUser(
        DDSROUTER_ECHO_DISCOVERY,
        "New endpoint discovered: " << endpoint_discovered << ".");
}

std::shared_ptr<IWriter> EchoDiscoveryParticipant::create_writer_(
        RealTopic)
{
    return std::make_shared<VoidWriter>();
}

std::shared_ptr<IReader> EchoDiscoveryParticipant::create_reader_(
        RealTopic)
{
    return std::make_shared<VoidReader>();
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
