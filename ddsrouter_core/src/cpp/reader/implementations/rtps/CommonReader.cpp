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
 * @file CommonReader.cpp
 */

#include <fastrtps/rtps/RTPSDomain.h>
#include <fastrtps/rtps/participant/RTPSParticipant.h>

#include <reader/implementations/rtps/CommonReader.hpp>
#include <ddsrouter_utils/exception/InitializationException.hpp>
#include <ddsrouter_utils/Log.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

using namespace eprosima::ddsrouter::core::types;

CommonReader::CommonReader(
        const ParticipantId& participant_id,
        const DdsTopic& topic,
        std::shared_ptr<PayloadPool> payload_pool,
        fastrtps::rtps::RTPSParticipant* rtps_participant,
        const fastrtps::rtps::HistoryAttributes& history_attributes,
        const fastrtps::rtps::ReaderAttributes& reader_attributes,
        const fastrtps::TopicAttributes& topic_attributes,
        const fastrtps::ReaderQos& reader_qos)
    : BaseReader(participant_id, topic, payload_pool)
    , rtps_participant_(rtps_participant)
{
    internal_entities_creation_(
        history_attributes,
        reader_attributes,
        topic_attributes,
        reader_qos);
}

CommonReader::~CommonReader()
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

    logInfo(DDSROUTER_RTPS_READER, "Deleting CommonReader created in Participant " <<
            participant_id_ << " for topic " << topic_);
}

void CommonReader::internal_entities_creation_(
        const fastrtps::rtps::HistoryAttributes& history_attributes,
        const fastrtps::rtps::ReaderAttributes& reader_attributes,
        const fastrtps::TopicAttributes& topic_attributes,
        const fastrtps::ReaderQos& reader_qos)
{
    // Copy reader attributes because fast needs it non const (do not ask why)
    fastrtps::rtps::ReaderAttributes non_const_reader_attributes = reader_attributes;

    // Create History
    rtps_history_ = new fastrtps::rtps::ReaderHistory(history_attributes);

    // Create CommonReader
    rtps_reader_ = fastrtps::rtps::RTPSDomain::createRTPSReader(
        rtps_participant_,
        non_const_reader_attributes,
        payload_pool_,
        rtps_history_);

    if (!rtps_reader_)
    {
        throw utils::InitializationException(
                  utils::Formatter() << "Error creating Simple RTPSReader for Participant " <<
                      participant_id_ << " in topic " << topic_ << ".");
    }

    // Set listener after entity creation to avoid SEGFAULT (produced when callback using rtps_reader_ is
    // invoked before the variable is fully set)
    rtps_reader_->setListener(this);

    // Register reader with topic
    if (!rtps_participant_->registerReader(rtps_reader_, topic_attributes, reader_qos))
    {
        // In case it fails, remove reader and throw exception
        fastrtps::rtps::RTPSDomain::removeRTPSReader(rtps_reader_);
        throw utils::InitializationException(utils::Formatter() << "Error registering topic " << topic_ <<
                      " for Simple RTPSReader in Participant " << participant_id_);
    }

    logInfo(DDSROUTER_RTPS_READER, "New CommonReader created in Participant " << participant_id_ << " for topic " <<
            topic_ << " with guid " << rtps_reader_->getGuid());
}

types::Guid CommonReader::guid() const noexcept
{
    return rtps_reader_->getGuid();
}

RecursiveTimedMutex& CommonReader::get_rtps_mutex() const noexcept
{
    return rtps_reader_->getMutex();
}

uint64_t CommonReader::get_unread_count() const noexcept
{
    return rtps_reader_->get_unread_count();
}

utils::ReturnCode CommonReader::take_(
        std::unique_ptr<DataReceived>& data) noexcept
{
    std::lock_guard<std::recursive_mutex> lock(rtps_mutex_);

    // Check if there is data available
    if (!(get_unread_count() > 0))
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

    // Check that the guid is consistent
    if (received_change->writerGUID == fastrtps::rtps::GUID_t::unknown())
    {
        logWarning(DDSROUTER_RTPS_COMMONREADER_LISTENER,
                "Error taking data without correct writer GUID.");

        // Remove the change in the History and release it in the reader
        rtps_reader_->getHistory()->remove_change(received_change);

        return utils::ReturnCode::RETCODE_ERROR;
    }

    // Check that the data is consistent
    // if (!(received_change->serializedPayload.max_size > 0))
    // {
    //     logWarning(DDSROUTER_RTPS_COMMONREADER_LISTENER,
    //             "Error taking data with length " << received_change->serializedPayload.length << ".");

    //     // Remove the change in the History and release it in the reader
    //     rtps_reader_->getHistory()->remove_change(received_change);

    //     return utils::ReturnCode::RETCODE_ERROR;
    // }

    // Store the new data that has arrived in the Track data
    fill_received_data_(received_change, data);

    logDebug(DDSROUTER_RTPS_COMMONREADER_LISTENER,
            "Data transmiting to track from Reader " << *this << " with payload " <<
            data->payload << " from remote writer " << data->properties.source_guid);

    // Remove the change in the History and release it in the reader
    rtps_reader_->getHistory()->remove_change(received_change);

    return utils::ReturnCode::RETCODE_OK;
}

void CommonReader::fill_received_data_(
    fastrtps::rtps::CacheChange_t* received_change,
    std::unique_ptr<types::DataReceived>& data_to_fill) const noexcept
{
    // Store the new data that has arrived in the Track data
    // Get the writer guid
    data_to_fill->properties.source_guid = received_change->writerGUID;
    // Get source timestamp
    data_to_fill->properties.source_timestamp = received_change->sourceTimestamp;
    // Get Participant receiver
    data_to_fill->properties.participant_receiver = participant_id_;

    // Store it in DDSRouter PayloadPool if size is bigger than 0
    // NOTE: in case of keyed topics an empty payload is possible
    if (received_change->serializedPayload.length > 0)
    {
        eprosima::fastrtps::rtps::IPayloadPool* payload_owner = received_change->payload_owner();
        payload_pool_->get_payload(
            received_change->serializedPayload,
            payload_owner,
            data_to_fill->payload);
    }

    // Set Instance Handle to data_to_fill
    if (topic_.keyed)
    {
        data_to_fill->properties.instanceHandle = received_change->instanceHandle;
    }

    data_to_fill->properties.write_params.set_value(received_change->write_params);

    data_to_fill->properties.origin_sequence_number = received_change->sequenceNumber;

    // Note: do not fill writer specific properties in this data from this kind of Readers.
    // Implement specific class for filling it.
}

void CommonReader::enable_() noexcept
{
    // If the topic is reliable, the reader will keep the samples received when it was disabled.
    // However, if the topic is best_effort, the reader will discard the samples received when it was disabled.
    if (topic_.topic_qos.value.is_reliable())
    {
        std::lock_guard<eprosima::fastrtps::RecursiveTimedMutex> lock(get_rtps_mutex());
        on_data_available_();
    }
}

bool CommonReader::come_from_this_participant_(
        const fastrtps::rtps::CacheChange_t* change) const noexcept
{
    return come_from_this_participant_(change->writerGUID);
}

bool CommonReader::come_from_this_participant_(
        const fastrtps::rtps::GUID_t guid) const noexcept
{
    return guid.guidPrefix == rtps_reader_->getGuid().guidPrefix;
}

fastrtps::rtps::HistoryAttributes CommonReader::history_attributes_(const types::DdsTopic& topic) noexcept
{
    fastrtps::rtps::HistoryAttributes att;
    att.memoryPolicy =
            eprosima::fastrtps::rtps::MemoryManagementPolicy_t::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;

    att.maximumReservedCaches = topic.topic_qos.value.history_depth;

    return att;
}

fastrtps::rtps::ReaderAttributes CommonReader::reader_attributes_(const types::DdsTopic& topic) noexcept
{
    fastrtps::rtps::ReaderAttributes att;

    // Set Durability
    att.endpoint.durabilityKind = topic.topic_qos.value.durability_qos;

    // Set Reliability
    att.endpoint.reliabilityKind = topic.topic_qos.value.reliability_qos;

    // Set if topic has key
    if (topic.keyed)
    {
        att.endpoint.topicKind = eprosima::fastrtps::rtps::WITH_KEY;
    }
    else
    {
        att.endpoint.topicKind = eprosima::fastrtps::rtps::NO_KEY;
    }

    // Ownership and Partitions are not part of RTPS, thus they are set in properties

    return att;
}

fastrtps::TopicAttributes CommonReader::topic_attributes_(const types::DdsTopic& topic) noexcept
{
    fastrtps::TopicAttributes att;

    // Set if topic has key
    if (topic.keyed)
    {
        att.topicKind = eprosima::fastrtps::rtps::WITH_KEY;
    }
    else
    {
        att.topicKind = eprosima::fastrtps::rtps::NO_KEY;
    }

    // Set Topic attributes
    att.topicName = topic.topic_name;
    att.topicDataType = topic.type_name;

    // Set Topic history attributes
    att.historyQos.kind = eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS;
    att.historyQos.depth = topic.topic_qos.value.history_depth;

    return att;
}

fastrtps::ReaderQos CommonReader::reader_qos_(const types::DdsTopic& topic) noexcept
{
    fastrtps::ReaderQos properties;

    // Set Durability
    properties.m_durability.kind =
        (topic.topic_qos.value.is_transient_local()
            ? eprosima::fastdds::dds::DurabilityQosPolicyKind_t::TRANSIENT_LOCAL_DURABILITY_QOS
            : eprosima::fastdds::dds::DurabilityQosPolicyKind_t::VOLATILE_DURABILITY_QOS);

    // Set Reliability
    properties.m_reliability.kind =
        (topic.topic_qos.value.is_reliable()
            ? eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS
            : eprosima::fastdds::dds::ReliabilityQosPolicyKind::BEST_EFFORT_RELIABILITY_QOS);

    // If topic with partitions, set this CommonReader in *
    if (topic.topic_qos.value.use_partitions)
    {
        properties.m_partition.push_back("*");
    }

    // If topic is with ownership
    properties.m_ownership.kind = topic.topic_qos.value.ownership_qos;

    return properties;
}

void CommonReader::onNewCacheChangeAdded(
        fastrtps::rtps::RTPSReader*,
        const fastrtps::rtps::CacheChange_t* const change) noexcept
{
    if (!come_from_this_participant_(change))
    {
        // Do not remove previous received changes so they can be read when the reader is enabled
        if (enabled_)
        {
            // Call Track callback (by calling BaseReader callback method)
            logDebug(DDSROUTER_RTPS_COMMONREADER_LISTENER,
                    "Data arrived to Reader " << *this << " with payload " << change->serializedPayload << " from " <<
                    change->writerGUID);
            on_data_available_();
        }
        else
        {
            // Remove received change if the CommonReader is disbled and the topic is not reliable
            // NOTE: this should be is_reliable and not is_transient_local for RPC sake
            if (!topic_.topic_qos.value.is_reliable())
            {
                rtps_reader_->getHistory()->remove_change((fastrtps::rtps::CacheChange_t*)change);
                logDebug(DDSROUTER_RTPS_COMMONREADER_LISTENER,
                        "Change removed from history");
            }
        }
    }
    else
    {
        logWarning(
            DDSROUTER_RTPS_COMMONREADER_LISTENER,
            "Ignoring data from this same Participant in reader " << *this << ".");

        // If it is a message from this Participant, do not send it forward and remove it
        // TODO: do this more elegant
        rtps_reader_->getHistory()->remove_change((fastrtps::rtps::CacheChange_t*)change);
    }
}

void CommonReader::onReaderMatched(
        fastrtps::rtps::RTPSReader*,
        fastrtps::rtps::MatchingInfo& info) noexcept
{
    if (!come_from_this_participant_(info.remoteEndpointGuid))
    {
        if (info.status == fastrtps::rtps::MatchingStatus::MATCHED_MATCHING)
        {
            logInfo(DDSROUTER_RTPS_COMMONREADER_LISTENER,
                    "Reader " << *this << " matched with a new Writer with guid " << info.remoteEndpointGuid);
        }
        else
        {
            logInfo(DDSROUTER_RTPS_COMMONREADER_LISTENER,
                    "Reader " << *this << " unmatched with Writer " << info.remoteEndpointGuid);
        }
    }
}

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
