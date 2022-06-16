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
 * @file IParticipant.cpp
 */

#include <memory>

#include <participant/IParticipant.hpp>
#include <ddsrouter_core/types/participant/ParticipantId.hpp>
#include <ddsrouter_utils/exception/InitializationException.hpp>
#include <ddsrouter_utils/Log.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

IParticipant::IParticipant(
        const types::ParticipantId& id)
    : id_(id)
    , configuration_(nullptr)
    , discovery_database_(nullptr)
{
}

IParticipant::IParticipant(
        const configuration::ParticipantConfiguration& participant_configuration,
        DiscoveryDatabase& discovery_database)
    : id_(participant_configuration.id())
    , configuration_(&participant_configuration)
    , discovery_database_(&discovery_database)
{
    logDebug(DDSROUTER_PARTICIPANT, "Creating Participant " << *this << ".");
}

IParticipant::~IParticipant()
{
    logDebug(DDSROUTER_PARTICIPANT, "Destroying Participant " << *this << ".");
}

const types::ParticipantId& IParticipant::id() const noexcept
{
    return id_;
}

std::pair<IWriter*, IReader*> IParticipant::register_topic(
        const types::RealTopic& topic,
        std::shared_ptr<fastrtps::rtps::IPayloadPool> payload_pool,
        DataForwardQueue& data_forward_queue)
{
    logInfo(DDSROUTER_BASEPARTICIPANT, "Creating writer and reader in Participant " << id() << " for topic " << topic);

    auto new_writer = create_writer_(topic, payload_pool);
    auto new_writer_raw = new_writer.get();
    writers_.insert( topic, std::move(new_writer));

    auto new_reader = create_reader_(topic, payload_pool, data_forward_queue);
    auto new_reader_raw = new_reader.get();
    readers_.insert( topic, std::move(new_reader));

    logInfo(DDSROUTER_BASEPARTICIPANT, "Created writer and reader in Participant " << id() << " for topic " << topic);

    return std::make_pair(
        new_writer_raw,
        new_reader_raw
        );
}

utils::ReturnCode IParticipant::enable_topic(
        const types::RealTopic& topic)
{

    auto topic_reader = readers_.get(topic);

    if (topic_reader)
    {
        return topic_reader->enable();
    }
    else
    {
        return utils::ReturnCode::RETCODE_NOT_ENABLED;
    }
}

utils::ReturnCode IParticipant::disable_topic(
        const types::RealTopic& topic)
{

    auto topic_reader = readers_.get(topic);

    if (topic_reader)
    {
        return topic_reader->disable();
    }
    else
    {
        return utils::ReturnCode::RETCODE_NOT_ENABLED;
    }
}

std::ostream& operator <<(
        std::ostream& os,
        const IParticipant& participant)
{
    os << "{" << participant.id().name() << ";" << participant.id().kind() << "}";
    return os;
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
