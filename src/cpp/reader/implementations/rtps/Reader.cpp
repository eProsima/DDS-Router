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
 * @file Reader.cpp
 */

#include <fastrtps/rtps/RTPSDomain.h>
#include <fastrtps/rtps/participant/RTPSParticipant.h>

#include <ddsrouter/reader/implementations/rtps/Reader.hpp>
#include <ddsrouter/exceptions/InitializationException.hpp>
#include <ddsrouter/types/Log.hpp>

namespace eprosima {
namespace ddsrouter {
namespace rtps {

Reader::Reader(
        const ParticipantId& participant_id,
        const RealTopic& topic,
        std::shared_ptr<PayloadPool> payload_pool,
        fastrtps::rtps::RTPSParticipant* rtps_participant)
    : BaseReader(participant_id, topic, payload_pool)
{
    // Create History
    fastrtps::rtps::HistoryAttributes history_att = history_attributes_();
    rtps_history_ = new fastrtps::rtps::ReaderHistory(history_att);

    // Create Reader
    fastrtps::rtps::ReaderAttributes reader_att = reader_attributes_();
    rtps_reader_ = fastrtps::rtps::RTPSDomain::createRTPSReader(
        rtps_participant,
        reader_att,
        rtps_history_,
        this);

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

    logInfo(DDSROUTER_RTPS_READER, "New Reader created in Participant " << participant_id_ << " for topic " <<
            topic << " with guid " << rtps_reader_->getGuid());
}

Reader::~Reader()
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

    logInfo(DDSROUTER_RTPS_READER, "Deleting Reader created in Participant " <<
            participant_id_ << " for topic " << topic_);
}

ReturnCode Reader::take_(
        std::unique_ptr<DataReceived>& data) noexcept
{
    std::lock_guard<std::recursive_mutex> lock(rtps_mutex_);

    // Check if there is data available
    if (!(rtps_reader_->get_unread_count() > 0))
    {
        return ReturnCode::RETCODE_NO_DATA;
    }

    fastrtps::rtps::CacheChange_t* received_change = nullptr;
    fastrtps::rtps::WriterProxy* wp = nullptr;

    // Read first change of the history
    if (!rtps_reader_->nextUntakenCache(&received_change, &wp))
    {
        // Error reading.
        return ReturnCode::RETCODE_ERROR;
    }

    // Store the new data that has arrived in the Track data
    // Get the writer guid
    data->source_guid = received_change->writerGUID;

    // Store it in DDSRouter PayloadPool
    payload_pool_->get_payload(received_change->serializedPayload, data->payload);

    logDebug(DDSROUTER_RTPS_READER_LISTENER,
            "Data transmiting to track from Reader " << *this << " with payload " <<
            received_change->serializedPayload << " from remote writer " << received_change->writerGUID);

    // Remove the change in the History and release it in the reader
    rtps_reader_->getHistory()->remove_change(received_change);
    rtps_reader_->releaseCache(received_change);

    return ReturnCode::RETCODE_OK;
}

bool Reader::come_from_this_participant_(
        const fastrtps::rtps::CacheChange_t* change) const noexcept
{
    return come_from_this_participant_(change->writerGUID);
}

bool Reader::come_from_this_participant_(
        const fastrtps::rtps::GUID_t guid) const noexcept
{
    return guid.guidPrefix == rtps_reader_->getGuid().guidPrefix;
}

fastrtps::rtps::HistoryAttributes Reader::history_attributes_() const noexcept
{
    fastrtps::rtps::HistoryAttributes att;
    return att;
}

fastrtps::rtps::ReaderAttributes Reader::reader_attributes_() const noexcept
{
    fastrtps::rtps::ReaderAttributes att;
    att.endpoint.durabilityKind = fastrtps::rtps::DurabilityKind_t::VOLATILE;
    att.endpoint.reliabilityKind = fastrtps::rtps::ReliabilityKind_t::BEST_EFFORT;
    return att;
}

fastrtps::TopicAttributes Reader::topic_attributes_() const noexcept
{
    fastrtps::TopicAttributes att;
    att.topicKind = fastrtps::rtps::TopicKind_t::NO_KEY;
    att.topicName = topic_.topic_name();
    att.topicDataType = topic_.topic_type();
    return att;
}

fastrtps::ReaderQos Reader::reader_qos_() const noexcept
{
    fastrtps::ReaderQos qos;
    qos.m_durability.kind = fastdds::dds::DurabilityQosPolicyKind::VOLATILE_DURABILITY_QOS;
    qos.m_reliability.kind = fastdds::dds::ReliabilityQosPolicyKind::BEST_EFFORT_RELIABILITY_QOS;
    return qos;
}

void Reader::onNewCacheChangeAdded(
        fastrtps::rtps::RTPSReader*,
        const fastrtps::rtps::CacheChange_t* const change) noexcept
{
    if (!come_from_this_participant_(change))
    {
        // Call Track callback (by calling BaseReader callback method)
        logDebug(DDSROUTER_RTPS_READER_LISTENER,
                "Data arrived to Reader " << *this << " with payload " << change->serializedPayload << " from " <<
                            change->writerGUID);
        on_data_available_();
    }
    else
    {
        // If it is a message from this Participant, do not send it forward and remove it
        // TODO: do this more elegant
        rtps_reader_->getHistory()->remove_change((fastrtps::rtps::CacheChange_t*)change);
    }
}

void Reader::onReaderMatched(
        fastrtps::rtps::RTPSReader*,
        fastrtps::rtps::MatchingInfo& info) noexcept
{
    if (!come_from_this_participant_(info.remoteEndpointGuid))
    {
        if (info.status == fastrtps::rtps::MatchingStatus::MATCHED_MATCHING)
        {
            logInfo(DDSROUTER_RTPS_READER_LISTENER,
                    "Reader " << *this << " matched with a new Writer with guid " << info.remoteEndpointGuid);
        }
        else
        {
            logInfo(DDSROUTER_RTPS_READER_LISTENER,
                    "Reader " << *this << " unmatched with Writer " << info.remoteEndpointGuid);
        }
    }
}

} /* namespace rtps */
} /* namespace ddsrouter */
} /* namespace eprosima */
