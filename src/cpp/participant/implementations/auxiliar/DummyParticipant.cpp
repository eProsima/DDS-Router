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
 * @file DummyParticipant.cpp
 */

#include <ddsrouter/participant/implementations/auxiliar/DummyParticipant.hpp>
#include <ddsrouter/reader/implementations/auxiliar/DummyReader.hpp>
#include <ddsrouter/writer/implementations/auxiliar/DummyWriter.hpp>
#include <ddsrouter/types/ParticipantType.hpp>

namespace eprosima {
namespace ddsrouter {

DummyParticipant::DummyParticipant(
        ParticipantConfiguration participant_configuration,
        std::shared_ptr<DiscoveryDatabase> discovery_database)
    : EchoParticipant(participant_configuration, discovery_database)
{
}

ParticipantType DummyParticipant::type() const
{
    return ParticipantType::DUMMY;
}

std::shared_ptr<IWriter> DummyParticipant::create_writer(
        RealTopic topic)
{
    return std::make_shared<DummyWriter>(id(), topic);
}

std::shared_ptr<IReader> DummyParticipant::create_reader(
        RealTopic topic)
{
    return std::make_shared<DummyReader>(id(), topic);
}

void DummyParticipant::add_discovered_endpoint(const Endpoint& new_endpoint)
{
    discovery_database_->add_or_modify_endpoint(new_endpoint);
}

Endpoint DummyParticipant::get_discovered_endpoint(const Guid& guid) const
{
    return discovery_database_->get_endpoint(guid);
}

} /* namespace ddsrouter */
} /* namespace eprosima */
