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
 * @file MockParticipant.cpp
 */

#include <ddsrouter_utils/exception/InconsistencyException.hpp>

#include <ddsrouter_core/types/participant/ParticipantKind.hpp>
#include <participant/implementations/auxiliar/MockParticipant.hpp>
#include <reader/implementations/auxiliar/MockReader.hpp>
#include <writer/implementations/auxiliar/MockWriter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

using namespace eprosima::ddsrouter::core::types;

std::mutex MockParticipant::static_mutex_;
std::map<ParticipantId, MockParticipant*> MockParticipant::participants_;

MockParticipant::MockParticipant(
        const configuration::ParticipantConfiguration participant_configuration,
        std::shared_ptr<PayloadPool> payload_pool,
        std::shared_ptr<DiscoveryDatabase> discovery_database)
    : BaseParticipant(participant_configuration, payload_pool, discovery_database)
{
    std::unique_lock<std::mutex> lock(static_mutex_);

    // Add this participant to the static list of all participants
    participants_[id()] = this;
}

MockParticipant::~MockParticipant()
{
    std::unique_lock<std::mutex> lock(static_mutex_);

    // Remove this participant from the static list of all participants
    participants_.erase(id());
}

std::shared_ptr<IWriter> MockParticipant::create_writer_(
        RealTopic topic)
{
    return std::make_shared<MockWriter>(id(), topic, payload_pool_);
}

std::shared_ptr<IReader> MockParticipant::create_reader_(
        RealTopic topic)
{
    return std::make_shared<MockReader>(id(), topic, payload_pool_);
}

MockReader* MockParticipant::get_reader(
        const types::RealTopic& topic)
{
    auto it = readers_.find(topic);
    if (it != readers_.end())
    {
        std::shared_ptr<MockReader> reader = std::dynamic_pointer_cast<MockReader>(it->second);
        return reader.get();
    }
    else
    {
        throw utils::InconsistencyException(
            STR_ENTRY << "No reader found in Participant " << id() << " for topic " << topic);
    }
}

MockWriter* MockParticipant::get_writer(
        const types::RealTopic& topic)
{
    auto it = writers_.find(topic);
    if (it != writers_.end())
    {
        std::shared_ptr<MockWriter> writer = std::dynamic_pointer_cast<MockWriter>(it->second);
        return writer.get();
    }
    else
    {
        throw utils::InconsistencyException(
            STR_ENTRY << "No writer found in Participant " << id() << " for topic " << topic);
    }
}

// void MockParticipant::simulate_data_reception(
//             const types::RealTopic& topic,
//             types::DataReceived&& data)
// {
//     auto it = readers_.find(topic);
//     if (it != readers_.end())
//     {
//         std::shared_ptr<MockReader> reader = std::dynamic_pointer_cast<MockReader>(it->second);
//         reader->simulate_data_reception(std::move(data));
//     }
//     else
//     {
//         throw utils::InconsistencyException(
//             STR_ENTRY << "No reader found in Participant " << id() << " for topic " << topic);
//     }
// }


// void MockParticipant::register_write_callback(
//         const types::RealTopic& topic,
//         const std::function<void(std::unique_ptr<DataReceived>&)>& callback)
// {
//     auto it = writers_.find(topic);
//     if (it != writers_.end())
//     {
//         std::shared_ptr<MockWriter> writer = std::dynamic_pointer_cast<MockWriter>(it->second);
//         return writer->register_write_callback(callback);
//     }
//     else
//     {
//         throw utils::InconsistencyException(
//             STR_ENTRY << "No writer found in Participant " << id() << " for topic " << topic);
//     }
// }

// void MockParticipant::wait_until_n_data_sent(
//         const types::RealTopic& topic,
//         uint32_t n) const
// {
//     auto it = writers_.find(topic);
//     if (it != writers_.end())
//     {
//         std::shared_ptr<MockWriter> writer = std::dynamic_pointer_cast<MockWriter>(it->second);
//         writer->wait_until_n_data_sent(n);
//     }
//     else
//     {
//         throw utils::InconsistencyException(
//             STR_ENTRY << "No writer found in Participant " << id() << " for topic " << topic);
//     }
// }

MockParticipant* MockParticipant::get_participant(
        ParticipantId id)
{
    auto it = participants_.find(id);

    if (it == participants_.end())
    {
        throw utils::InconsistencyException(
            STR_ENTRY << "No Participant found with id " << id);
    }
    else
    {
        return it->second;
    }
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
