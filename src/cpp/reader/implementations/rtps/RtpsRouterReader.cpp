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
 * @file RTPSRouterReader.cpp
 */

#include <fastrtps/rtps/RTPSDomain.h>
#include <fastrtps/rtps/participant/RTPSParticipant.h>

#include <ddsrouter/reader/implementations/rtps/RTPSRouterReader.hpp>
#include <ddsrouter/exceptions/InitializationException.hpp>
#include <ddsrouter/types/Log.hpp>

namespace eprosima {
namespace ddsrouter {

RTPSRouterReader::RTPSRouterReader(
        const ParticipantId& participant_id,
        const RealTopic& topic,
        std::shared_ptr<PayloadPool> payload_pool,
        fastrtps::rtps::RTPSParticipant* rtps_participant)
    : BaseReader(participant_id_, topic_, payload_pool_)
{
    // Create History
    fastrtps::rtps::HistoryAttributes history_att = history_attributes_();
    rtps_history_ = new fastrtps::rtps::ReaderHistory(history_att);

    // Create Reader
    fastrtps::rtps::ReaderAttributes reader_att = reader_attributes_();
    rtps_reader_ = fastrtps::rtps::RTPSDomain::createRTPSReader(rtps_participant, reader_att, rtps_history_, nullptr);

    if (!rtps_reader_)
    {
        throw InitializationException(utils::Formatter() << "Error creating Simple RTPSReader for Participant " <<
            participant_id << " in topic " << topic << ".");
    }

    // Register reader with topic
    fastrtps::TopicAttributes topic_att = topic_attributes_();
    fastrtps::ReaderQos reader_qos = reader_qos_();

    if (!rtps_participant->registerReader(rtps_reader_, topic_att, reader_qos))
    {
        // In case it fails, remove reader and throw exception
        fastrtps::rtps::RTPSDomain::removeRTPSReader(rtps_reader_);
        throw InitializationException(utils::Formatter() << "Error registering topic " << topic <<
            " for Simple RTPSReader in Participant " << participant_id);
    }

    logInfo(DDSROUTER_RTPS_WRITER, "New Reader created in Participant " << participant_id_ << " for topic " <<
        topic << " with guid " << rtps_reader_->getGuid());
}

RTPSRouterReader::~RTPSRouterReader()
{
    // This variables should be set, otherwise the creation should have fail
    // Anyway, the if case is used for safety reasons

    // Delete reader
    if (rtps_reader_)
    {
        fastrtps::rtps::RTPSDomain::removeRTPSReader(rtps_reader_);
    }

    // Delete History
    if (rtps_history_)
    {
        delete rtps_history_;
    }

    logInfo(DDSROUTER_RTPS_WRITER, "Deleting Reader created in Participant " <<
        participant_id_ << " for topic " << topic_);
}

ReturnCode RTPSRouterReader::take_(
        std::unique_ptr<DataReceived>& data) noexcept
{
    // TODO
    return ReturnCode::RETCODE_NO_DATA;
}

fastrtps::rtps::HistoryAttributes RTPSRouterReader::history_attributes_() const noexcept
{
    fastrtps::rtps::HistoryAttributes att;
    return att;
}

fastrtps::rtps::ReaderAttributes RTPSRouterReader::reader_attributes_() const noexcept
{
    fastrtps::rtps::ReaderAttributes att;
    att.endpoint.durabilityKind = eprosima::fastrtps::rtps::DurabilityKind_t::VOLATILE;
    att.endpoint.reliabilityKind = eprosima::fastrtps::rtps::ReliabilityKind_t::BEST_EFFORT;
    return att;
}

fastrtps::TopicAttributes RTPSRouterReader::topic_attributes_() const noexcept
{
    fastrtps::TopicAttributes att;
    att.topicKind = eprosima::fastrtps::rtps::TopicKind_t::NO_KEY;
    att.topicName = topic_.topic_name();
    att.topicDataType = topic_.topic_type();
    return att;
}

fastrtps::ReaderQos RTPSRouterReader::reader_qos_() const noexcept
{
    fastrtps::ReaderQos qos;
    qos.m_durability.kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::VOLATILE_DURABILITY_QOS;
    qos.m_reliability.kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::BEST_EFFORT_RELIABILITY_QOS;
    return qos;
}

} /* namespace ddsrouter */
} /* namespace eprosima */
