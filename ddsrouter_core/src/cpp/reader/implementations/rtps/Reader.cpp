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

#include <reader/implementations/rtps/Reader.hpp>
#include <ddsrouter_utils/exception/InitializationException.hpp>
#include <ddsrouter_utils/Log.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

using namespace eprosima::ddsrouter::core::types;

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
        payload_pool_,
        rtps_history_);

    // Set listener after entity creation to avoid SEGFAULT (produced when callback using rtps_reader_ is
    // invoked before the variable is fully set)
    rtps_reader_->setListener(this);

    if (!rtps_reader_)
    {
        throw utils::InitializationException(
                  utils::Formatter() << "Error creating Simple RTPSReader for Participant " <<
                      participant_id << " in topic " << topic << ".");
    }

    // Register reader with topic
    fastrtps::TopicAttributes topic_att = topic_attributes_();
    fastrtps::ReaderQos reader_qos = reader_qos_();

    if (!rtps_participant->registerReader(rtps_reader_, topic_att, reader_qos))
    {
        // In case it fails, remove reader and throw exception
        fastrtps::rtps::RTPSDomain::removeRTPSReader(rtps_reader_);
        throw utils::InitializationException(utils::Formatter() << "Error registering topic " << topic <<
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

utils::ReturnCode Reader::take_(
        std::unique_ptr<DataReceived>& data) noexcept
{
    std::lock_guard<std::recursive_mutex> lock(rtps_mutex_);

    // Check if there is data available
    if (!(rtps_reader_->get_unread_count() > 0))
    {
        return utils::ReturnCode::RETCODE_NO_DATA;
    }

    fastrtps::rtps::CacheChange_t* received_change = nullptr;
    fastrtps::rtps::WriterProxy* wp = nullptr;

    // Read first change of the history
    if (!rtps_reader_->nextUntakenCache(&received_change, &wp))
    {
        // Error reading.
        return utils::ReturnCode::RETCODE_ERROR;
    }

    // Check that the data is consistent
    if (!(received_change->serializedPayload.max_size > 0))
    {
        logWarning(DDSROUTER_RTPS_READER_LISTENER,
                "Error taking data with length " << received_change->serializedPayload.length << ".");

        // Remove the change in the History and release it in the reader
        rtps_reader_->getHistory()->remove_change(received_change);

        return utils::ReturnCode::RETCODE_ERROR;
    }

    // Check that the guid is consistent
    if (received_change->writerGUID == fastrtps::rtps::GUID_t::unknown())
    {
        logWarning(DDSROUTER_RTPS_READER_LISTENER,
                "Error taking data without correct writer GUID.");

        // Remove the change in the History and release it in the reader
        rtps_reader_->getHistory()->remove_change(received_change);

        return utils::ReturnCode::RETCODE_ERROR;
    }

    // Store the new data that has arrived in the Track data
    // Get the writer guid
    data->source_guid = received_change->writerGUID;

    // Store it in DDSRouter PayloadPool
    eprosima::fastrtps::rtps::IPayloadPool* payload_owner = received_change->payload_owner();
    payload_pool_->get_payload(
        received_change->serializedPayload,
        payload_owner,
        data->payload);

    logDebug(DDSROUTER_RTPS_READER_LISTENER,
            "Data transmiting to track from Reader " << *this << " with payload " <<
            received_change->serializedPayload << " from remote writer " << received_change->writerGUID);

    // Remove the change in the History and release it in the reader
    rtps_reader_->getHistory()->remove_change(received_change);

    return utils::ReturnCode::RETCODE_OK;
}

void Reader::enable_() noexcept
{
    // TODO: refactor with the transparency module
    // If the topic is reliable, the reader will keep the samples received when it was disabled.
    // However, if the topic is best_effort, the reader will discard the samples received when it was disabled.
    on_data_available_();
}

bool Reader::accept_message_from_this_source_(
        const fastrtps::rtps::CacheChange_t* change) const noexcept
{
    return accept_message_from_this_source_(change->writerGUID);
}

bool Reader::accept_message_from_this_source_(
        const fastrtps::rtps::GUID_t guid) const noexcept
{
    return true;
}

fastrtps::rtps::HistoryAttributes Reader::history_attributes_() const noexcept
{
    fastrtps::rtps::HistoryAttributes att;
    att.memoryPolicy =
            eprosima::fastrtps::rtps::MemoryManagementPolicy_t::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
    return att;
}

fastrtps::rtps::ReaderAttributes Reader::reader_attributes_() const noexcept
{
    fastrtps::rtps::ReaderAttributes att;

    if (topic_.topic_reliable())
    {
        att.endpoint.reliabilityKind = fastrtps::rtps::ReliabilityKind_t::RELIABLE;
        att.endpoint.durabilityKind = fastrtps::rtps::DurabilityKind_t::TRANSIENT_LOCAL;
    }
    else
    {
        att.endpoint.reliabilityKind = fastrtps::rtps::ReliabilityKind_t::BEST_EFFORT;
        att.endpoint.durabilityKind = fastrtps::rtps::DurabilityKind_t::VOLATILE;
    }

    if (topic_.topic_with_key())
    {
        att.endpoint.topicKind = eprosima::fastrtps::rtps::WITH_KEY;
    }
    else
    {
        att.endpoint.topicKind = eprosima::fastrtps::rtps::NO_KEY;
    }
    return att;
}

fastrtps::TopicAttributes Reader::topic_attributes_() const noexcept
{
    fastrtps::TopicAttributes att;
    if (topic_.topic_with_key())
    {
        att.topicKind = eprosima::fastrtps::rtps::WITH_KEY;
    }
    else
    {
        att.topicKind = eprosima::fastrtps::rtps::NO_KEY;
    }
    att.topicName = topic_.topic_name();
    att.topicDataType = topic_.topic_type();
    return att;
}

fastrtps::ReaderQos Reader::reader_qos_() const noexcept
{
    fastrtps::ReaderQos qos;

    if (topic_.topic_reliable())
    {
        qos.m_reliability.kind = fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
        qos.m_durability.kind = fastdds::dds::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
    }
    else
    {
        qos.m_reliability.kind = fastdds::dds::ReliabilityQosPolicyKind::BEST_EFFORT_RELIABILITY_QOS;
        qos.m_durability.kind = fastdds::dds::DurabilityQosPolicyKind::VOLATILE_DURABILITY_QOS;
    }

    return qos;
}

void Reader::onNewCacheChangeAdded(
        fastrtps::rtps::RTPSReader*,
        const fastrtps::rtps::CacheChange_t* const change) noexcept
{
    if (accept_message_from_this_source_(change))
    {
        // Do not remove previous received changes so they can be read when the reader is enabled
        if (enabled_)
        {
            // Call Track callback (by calling BaseReader callback method)
            logDebug(DDSROUTER_RTPS_READER_LISTENER,
                    "Data arrived to Reader " << *this << " with payload " << change->serializedPayload << " from " <<
                    change->writerGUID);
            on_data_available_();
        }
        else
        {
            // Remove received change if the Reader is disbled and the topic is not reliable
            if (!topic_.topic_reliable())
            {
                rtps_reader_->getHistory()->remove_change((fastrtps::rtps::CacheChange_t*)change);
            }
        }
    }
    else
    {
        logWarning(
            DDSROUTER_RTPS_READER_LISTENER,
            "Ignoring data from " << change->writerGUID << " in reader " << *this << ".");

        // If it is a message from this Participant, do not send it forward and remove it
        // TODO: do this more elegant
        rtps_reader_->getHistory()->remove_change((fastrtps::rtps::CacheChange_t*)change);
    }
}

void Reader::onReaderMatched(
        fastrtps::rtps::RTPSReader*,
        fastrtps::rtps::MatchingInfo& info) noexcept
{
    if (accept_message_from_this_source_(info.remoteEndpointGuid))
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
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
