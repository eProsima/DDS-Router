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

#include <memory>

#include <fastrtps/rtps/participant/RTPSParticipant.h>
#include <fastrtps/rtps/RTPSDomain.h>

#include <ddsrouter/participant/implementations/rtps/SimpleParticipant.hpp>
#include <ddsrouter/reader/implementations/rtps/Reader.hpp>
#include <ddsrouter/writer/implementations/rtps/Writer.hpp>
#include <ddsrouter/exceptions/InitializationException.hpp>

namespace eprosima {
namespace ddsrouter {
namespace rtps {

SimpleParticipant::SimpleParticipant(
        const ParticipantConfiguration& participant_configuration,
        std::shared_ptr<PayloadPool> payload_pool,
        std::shared_ptr<DiscoveryDatabase> discovery_database)
    : BaseParticipant(participant_configuration, payload_pool, discovery_database)
{
    DomainId domain = configuration_.domain();
    fastrtps::rtps::RTPSParticipantAttributes params = SimpleParticipant::participant_attributes();

    rtps_participant_ = fastrtps::rtps::RTPSDomain::createParticipant(domain(), params);
    if (!rtps_participant_)
    {
        throw InitializationException(utils::Formatter() << "Error creating Simple RTPS Participant " << id());
    }

    logInfo(DDSROUTER_RTPS_READER, "New Participant created with id " << id() << " in domain " <<
            domain << " with guid " << rtps_participant_->getGuid());
}

SimpleParticipant::~SimpleParticipant()
{
    if (rtps_participant_)
    {
        fastrtps::rtps::RTPSDomain::removeRTPSParticipant(rtps_participant_);
    }
}

std::shared_ptr<IWriter> SimpleParticipant::create_writer_(
        RealTopic topic)
{
    return std::make_shared<Writer>(id(), topic, payload_pool_, rtps_participant_);
}

std::shared_ptr<IReader> SimpleParticipant::create_reader_(
        RealTopic topic)
{
    return std::make_shared<Reader>(id(), topic, payload_pool_, rtps_participant_);
}

fastrtps::rtps::RTPSParticipantAttributes SimpleParticipant::participant_attributes() const noexcept
{
    fastrtps::rtps::RTPSParticipantAttributes params;
    return params;
}

} /* namespace rtps */
} /* namespace ddsrouter */
} /* namespace eprosima */
