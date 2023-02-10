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


#include <cpp_utils/Log.hpp>

#include <ddspipe_participants/configuration/EchoParticipantConfiguration.hpp>
#include <ddspipe_participants/participant/auxiliar/EchoParticipant.hpp>
#include <ddspipe_participants/reader/auxiliar/BlankReader.hpp>
#include <ddspipe_participants/writer/auxiliar/EchoWriter.hpp>
#include <ddspipe_participants/writer/auxiliar/BlankWriter.hpp>


namespace eprosima {
namespace ddspipe {
namespace participants {

using namespace eprosima::ddsrouter::core::types;

EchoParticipant::EchoParticipant(
        std::shared_ptr<EchoParticipantConfiguration> participant_configuration,
        std::shared_ptr<core::DiscoveryDatabase> discovery_database)
    : BlankParticipant(participant_configuration->id)
    , configuration_(participant_configuration)
{
    logDebug(DDSROUTER_TRACK, "Creating Echo Participant : " << configuration_->id << " .");

    if (configuration_->echo_discovery)
    {
        // Register in Discovery DB a callback to be notified each time an endpoint is discovered
        discovery_database->add_endpoint_discovered_callback(
            [this](const Endpoint& endpoint_discovered)
            {
                this->echo_discovery(endpoint_discovered);
            });
    }
}

void EchoParticipant::echo_discovery(
        Endpoint endpoint_discovered) const noexcept
{
    // TODO write this in a way that is efficient and easy to read and allow verbose option
    logUser(
        DDSROUTER_ECHO_DISCOVERY,
        "New endpoint discovered: " << endpoint_discovered << ".");
}

std::shared_ptr<core::IWriter> EchoParticipant::create_writer(
        DistributedTopic topic)
{
    if (configuration_->echo_data)
    {
        return std::make_shared<EchoWriter>(
            topic,
            configuration_->verbose);
    }
    else
    {
        return std::make_shared<BlankWriter>();
    }
}

} /* namespace participants */
} /* namespace ddspipe */
} /* namespace eprosima */
