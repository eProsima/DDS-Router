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

#include <ddsrouter_core/types/participant/ParticipantKind.hpp>

#include <participant/implementations/auxiliar/DummyParticipant.hpp>
#include <reader/implementations/auxiliar/DummyReader.hpp>
#include <writer/implementations/auxiliar/DummyWriter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

using namespace eprosima::ddsrouter::core::types;

std::mutex DummyParticipant::static_mutex_;
std::map<ParticipantId, DummyParticipant*> DummyParticipant::participants_;

DummyParticipant::DummyParticipant(
        const configuration::ParticipantConfiguration participant_configuration,
        std::shared_ptr<PayloadPool> payload_pool,
        std::shared_ptr<DiscoveryDatabase> discovery_database)
    : BaseParticipant(participant_configuration, payload_pool, discovery_database)
{
    std::unique_lock<std::mutex> lock(static_mutex_);

    // Add this participant to the static list of all participants
    participants_[id()] = this;
}

DummyParticipant::~DummyParticipant()
{
    std::unique_lock<std::mutex> lock(static_mutex_);

    // Remove this participant from the static list of all participants
    participants_.erase(id());
}

std::shared_ptr<IWriter> DummyParticipant::create_writer_(
        RealTopic topic)
{
    return std::make_shared<DummyWriter>(id(), topic, payload_pool_);
}

std::shared_ptr<IReader> DummyParticipant::create_reader_(
        RealTopic topic)
{
    return std::make_shared<DummyReader>(id(), topic, payload_pool_);
}

void DummyParticipant::simulate_discovered_endpoint(
        const Endpoint& new_endpoint)
{
    discovery_database_->add_endpoint(new_endpoint);
}

Endpoint DummyParticipant::get_discovered_endpoint(
        const Guid& guid) const
{
    return discovery_database_->get_endpoint(guid);
}

void DummyParticipant::simulate_data_reception(
        RealTopic topic,
        DummyDataReceived data)
{
    auto it = readers_.find(topic);
    if (it != readers_.end())
    {
        std::shared_ptr<DummyReader> reader = std::dynamic_pointer_cast<DummyReader>(it->second);
        reader->simulate_data_reception(data);
    }
}

std::vector<DummyDataStored> DummyParticipant::get_data_that_should_have_been_sent(
        RealTopic topic)
{
    auto it = writers_.find(topic);
    if (it != writers_.end())
    {
        std::shared_ptr<DummyWriter> writer = std::dynamic_pointer_cast<DummyWriter>(it->second);
        return writer->get_data_that_should_have_been_sent();
    }
    else
    {
        return std::vector<DummyDataStored>();
    }
}

void DummyParticipant::wait_until_n_data_sent(
        RealTopic topic,
        uint16_t n) const noexcept
{
    auto it = writers_.find(topic);
    if (it != writers_.end())
    {
        std::shared_ptr<DummyWriter> writer = std::dynamic_pointer_cast<DummyWriter>(it->second);
        writer->wait_until_n_data_sent(n);
    }
}

DummyParticipant* DummyParticipant::get_participant(
        ParticipantId id)
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

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
