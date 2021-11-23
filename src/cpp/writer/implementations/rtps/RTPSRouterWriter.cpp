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
 * @file RTPSRouterWriter.cpp
 */

#include <fastrtps/rtps/RTPSDomain.h>
#include <fastrtps/rtps/participant/RTPSParticipant.h>

#include <ddsrouter/writer/implementations/rtps/RTPSRouterWriter.hpp>
#include <ddsrouter/exceptions/InitializationException.hpp>
#include <ddsrouter/types/Log.hpp>

namespace eprosima {
namespace ddsrouter {

RTPSRouterWriter::RTPSRouterWriter(
        const ParticipantId& participant_id,
        const RealTopic& topic,
        std::shared_ptr<PayloadPool> payload_pool,
        fastrtps::rtps::RTPSParticipant* rtps_participant)
    : BaseWriter(participant_id, topic, payload_pool)
{
    // Create History
    fastrtps::rtps::HistoryAttributes history_att = history_attributes_();
    rtps_history_ = new fastrtps::rtps::WriterHistory(history_att);

    // Create Writer
    fastrtps::rtps::WriterAttributes writer_att = writer_attributes_();
    rtps_writer_ = fastrtps::rtps::RTPSDomain::createRTPSWriter(rtps_participant, writer_att, rtps_history_, nullptr);

    if (!rtps_writer_)
    {
        throw InitializationException(utils::Formatter() << "Error creating Simple RTPSWriter for Participant " <<
            participant_id << " in topic " << topic << ".");
    }

    // Register writer with topic
    fastrtps::TopicAttributes topic_att = topic_attributes_();
    fastrtps::WriterQos writer_qos = writer_qos_();

    if (!rtps_participant->registerWriter(rtps_writer_, topic_att, writer_qos))
    {
        // In case it fails, remove writer and throw exception
        fastrtps::rtps::RTPSDomain::removeRTPSWriter(rtps_writer_);
        throw InitializationException(utils::Formatter() << "Error registering topic " << topic <<
            " for Simple RTPSWriter in Participant " << participant_id);
    }

    logInfo(DDSROUTER_RTPS_WRITER, "New Writer created in Participant " << participant_id_ << " for topic " <<
        topic << " with guid " << rtps_writer_->getGuid());
}

RTPSRouterWriter::~RTPSRouterWriter()
{
    // This variables should be set, otherwise the creation should have fail
    // Anyway, the if case is used for safety reasons

    // Delete writer
    if (rtps_writer_)
    {
        fastrtps::rtps::RTPSDomain::removeRTPSWriter(rtps_writer_);
    }

    // Delete History
    if (rtps_history_)
    {
        delete rtps_history_;
    }

    logInfo(DDSROUTER_RTPS_WRITER, "Deleting Writer created in Participant " <<
        participant_id_ << " for topic " << topic_);
}

// Specific enable/disable do not need to be implemented
ReturnCode RTPSRouterWriter::write_(
        std::unique_ptr<DataReceived>& data) noexcept
{
    // TODO
    return ReturnCode::RETCODE_OK;
}

fastrtps::rtps::HistoryAttributes RTPSRouterWriter::history_attributes_() const noexcept
{
    fastrtps::rtps::HistoryAttributes att;
    return att;
}

fastrtps::rtps::WriterAttributes RTPSRouterWriter::writer_attributes_() const noexcept
{
    fastrtps::rtps::WriterAttributes att;
    att.endpoint.durabilityKind = eprosima::fastrtps::rtps::DurabilityKind_t::TRANSIENT_LOCAL;
    att.endpoint.reliabilityKind = eprosima::fastrtps::rtps::ReliabilityKind_t::RELIABLE;
    return att;
}

fastrtps::TopicAttributes RTPSRouterWriter::topic_attributes_() const noexcept
{
    fastrtps::TopicAttributes att;
    att.topicKind = eprosima::fastrtps::rtps::TopicKind_t::NO_KEY;
    att.topicName = topic_.topic_name();
    att.topicDataType = topic_.topic_type();
    return att;
}

fastrtps::WriterQos RTPSRouterWriter::writer_qos_() const noexcept
{
    fastrtps::WriterQos qos;
    qos.m_durability.kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
    qos.m_reliability.kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    return qos;
}

} /* namespace ddsrouter */
} /* namespace eprosima */
