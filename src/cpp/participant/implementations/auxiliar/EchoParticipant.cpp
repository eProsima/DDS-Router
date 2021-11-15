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
 * @file EchoParticipant.cpp
 */

#include <ddsrouter/participant/implementations/auxiliar/EchoParticipant.hpp>
#include <ddsrouter/reader/implementations/auxiliar/VoidReader.hpp>
#include <ddsrouter/writer/implementations/auxiliar/EchoWriter.hpp>
#include <ddsrouter/types/ParticipantType.hpp>

namespace eprosima {
namespace ddsrouter {

EchoParticipant::EchoParticipant(
        ParticipantConfiguration participant_configuration,
        std::shared_ptr<DiscoveryDatabase> discovery_database)
    : VoidParticipant(participant_configuration)
    , discovery_database_(discovery_database)
{
}

ParticipantType EchoParticipant::type() const
{
    return ParticipantType::ECHO;
}

std::shared_ptr<IWriter> EchoParticipant::create_writer(
        RealTopic topic)
{
    return std::make_shared<EchoWriter>(id(), topic);
}

} /* namespace ddsrouter */
} /* namespace eprosima */
