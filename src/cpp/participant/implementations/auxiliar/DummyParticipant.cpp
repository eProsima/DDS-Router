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

std::map<ParticipantId, std::shared_ptr<DummyParticipant>> DummyParticipant::participants_;

DummyParticipant::DummyParticipant(
        ParticipantConfiguration participant_configuration,
        std::shared_ptr<DiscoveryDatabase> discovery_database)
    : EchoParticipant(participant_configuration, discovery_database)
{
    participants_[id()] = std::shared_ptr<DummyParticipant>(this);
}

ParticipantType DummyParticipant::type() const
{
    return ParticipantType::DUMMY;
}

std::shared_ptr<IWriter> DummyParticipant::create_writer(
        RealTopic topic)
{
    std::shared_ptr<DummyWriter> writer = std::make_shared<DummyWriter>(id(), topic);

    writers_[topic] = writer;

    return writer;
}

std::shared_ptr<IReader> DummyParticipant::create_reader(
        RealTopic topic)
{
    std::shared_ptr<DummyReader> reader = std::make_shared<DummyReader>(id(), topic);

    readers_[topic] = reader;

    return reader;
}

void DummyParticipant::add_discovered_endpoint(const Endpoint& new_endpoint)
{
    discovery_database_->add_or_modify_endpoint(new_endpoint);
}

Endpoint DummyParticipant::get_discovered_endpoint(const Guid& guid) const
{
    return discovery_database_->get_endpoint(guid);
}

void DummyParticipant::add_message_to_send(RealTopic topic, DataToSend data)
{
    auto it = readers_.find(topic);
    if (it == readers_.end())
    {
        it->second->add_message_to_send(data);
    }
}

std::vector<DataStoraged> DummyParticipant::data_received_ref(RealTopic topic)
{
    auto it = writers_.find(topic);
    if (it == writers_.end())
    {
        return it->second->data_received_ref();
    }
    else
    {
        return std::vector<DataStoraged>();
    }
}

std::shared_ptr<DummyParticipant> DummyParticipant::get_participant(ParticipantId id)
{
    auto it = participants_.find(id);

    if (it == participants_.end())
    {
        return nullptr;
    }
    else
    {
        return it->second;
    }
}

} /* namespace ddsrouter */
} /* namespace eprosima */
