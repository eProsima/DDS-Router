// Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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



#include <ddsrouter_core/participants/participant/auxiliar/MockParticipant.hpp>
#include <ddsrouter_core/participants/reader/auxiliar/MockReader.hpp>
#include <ddsrouter_core/participants/writer/auxiliar/MockWriter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace participants {

using namespace eprosima::ddsrouter::core::types;

MockParticipant::MockParticipant(
        const std::shared_ptr<ParticipantConfiguration>& participant_configuration,
        const std::shared_ptr<core::PayloadPool>& payload_pool,
        const std::shared_ptr<core::DiscoveryDatabase>& discovery_database)
    : BaseParticipant(participant_configuration, payload_pool, discovery_database)
{
    // Do nothing
}

std::shared_ptr<core::IWriter> MockParticipant::create_writer_(
        DdsTopic topic)
{
    return std::make_shared<MockWriter>(id(), topic, payload_pool_);
}

std::shared_ptr<core::IReader> MockParticipant::create_reader_(
        DdsTopic topic)
{
    return std::make_shared<MockReader>(id(), topic, payload_pool_);
}

std::shared_ptr<MockReader> MockParticipant::get_reader(
        core::types::DdsTopic topic)
{
    std::lock_guard<std::recursive_mutex> _(mutex_);
    auto it = readers_.find(topic);
    if (it != readers_.end())
    {
        return it.second;
    }
    return std::shared_ptr<MockReader>();
}

std::vector<MockDataStored> MockParticipant::get_writer(
        core::types::DdsTopic topic)
{
    std::lock_guard<std::recursive_mutex> _(mutex_);
    auto it = writers_.find(topic);
    if (it != writers_.end())
    {
        return it.second;
    }
    return std::shared_ptr<MockWriter>();
}

} /* namespace participants */
} /* namespace ddsrouter */
} /* namespace eprosima */
