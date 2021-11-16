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
 * @file SingletonDummyParticipant.cpp
 */

#include <ddsrouter/participant/implementations/auxiliar/SingletonDummyParticipant.hpp>
#include <ddsrouter/types/ParticipantId.hpp>
#include <ddsrouter/types/ParticipantType.hpp>

namespace eprosima {
namespace ddsrouter {

std::shared_ptr<SingletonDummyParticipant> SingletonDummyParticipant::get_instance()
{
    static std::shared_ptr<SingletonDummyParticipant> instance =
        std::shared_ptr<SingletonDummyParticipant>(new SingletonDummyParticipant());
    return instance;
}

ParticipantId SingletonDummyParticipant::id() const
{
    return participant_configuration_.id();
}

ParticipantType SingletonDummyParticipant::type() const
{
    return ParticipantType::SINGLETON_DUMMY;
}

std::shared_ptr<IWriter> SingletonDummyParticipant::create_writer(
        RealTopic topic)
{
    std::shared_ptr<DummyWriter> new_writer = std::make_shared<DummyWriter>(id(), topic);

    writers_[topic] = new_writer;

    return new_writer;
}

std::shared_ptr<IReader> SingletonDummyParticipant::create_reader(
        RealTopic topic)
{
    std::shared_ptr<DummyReader> new_reader = std::make_shared<DummyReader>(id(), topic);

    readers_[topic] = new_reader;

    return new_reader;
}

void SingletonDummyParticipant::add_discovered_endpoint(const Endpoint& new_endpoint)
{
    discovery_database_->add_or_modify_endpoint(new_endpoint);
}

Endpoint SingletonDummyParticipant::get_discovered_endpoint(const Guid& guid) const
{
    return discovery_database_->get_endpoint(guid);
}

void SingletonDummyParticipant::add_message_to_send(RealTopic topic, DataToSend data)
{
    readers_[topic]->add_message_to_send(data);
}

std::vector<DataStoraged>& SingletonDummyParticipant::data_received_ref(RealTopic topic)
{
    return writers_[topic]->data_received_ref();
}

void SingletonDummyParticipant::set_configuration(ParticipantConfiguration participant_configuration)
{
    participant_configuration_ = participant_configuration;
}

void SingletonDummyParticipant::set_discovery_database(std::shared_ptr<DiscoveryDatabase> discovery_database)
{
    discovery_database_ = discovery_database;
}

SingletonDummyParticipant::SingletonDummyParticipant()
    : participant_configuration_(ParticipantId(), RawConfiguration())
{
}

} /* namespace ddsrouter */
} /* namespace eprosima */
