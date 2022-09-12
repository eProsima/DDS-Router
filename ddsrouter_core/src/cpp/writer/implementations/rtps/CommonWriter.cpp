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
 * @file CommonWriter.cpp
 */

#include <fastrtps/rtps/RTPSDomain.h>
#include <fastrtps/rtps/participant/RTPSParticipant.h>
#include <fastrtps/rtps/common/CacheChange.h>

#include <ddsrouter_utils/exception/InitializationException.hpp>
#include <ddsrouter_utils/Log.hpp>
#include <efficiency/cache_change/CacheChangePool.hpp>
#include <writer/implementations/rtps/CommonWriter.hpp>
#include <writer/implementations/rtps/filter/RepeaterDataFilter.hpp>
#include <writer/implementations/rtps/filter/SelfDataFilter.hpp>
#include <types/dds/RouterCacheChange.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

using namespace eprosima::ddsrouter::core::types;

CommonWriter::CommonWriter(
        const ParticipantId& participant_id,
        const DdsTopic& topic,
        std::shared_ptr<PayloadPool> payload_pool,
        fastrtps::rtps::RTPSParticipant* rtps_participant,
        const bool repeater,
        const fastrtps::rtps::HistoryAttributes& history_attributes,
        const fastrtps::rtps::WriterAttributes& writer_attributes,
        const fastrtps::TopicAttributes& topic_attributes,
        const fastrtps::WriterQos& writer_qos,
        const utils::PoolConfiguration& pool_configuration)
    : BaseWriter(participant_id, topic, payload_pool)
    , repeater_(repeater)
    , rtps_participant_(rtps_participant)
{
    internal_entities_creation_(
        history_attributes,
        writer_attributes,
        topic_attributes,
        writer_qos,
        pool_configuration);
}

CommonWriter::~CommonWriter()
{
    // This variables should be set, otherwise the creation should have fail
    // Anyway, the if case is used for safety reasons

    // Delete writer
    if (rtps_writer_)
    {
        // Delete the CommonWriter the History is cleaned
        fastrtps::rtps::RTPSDomain::removeRTPSWriter(rtps_writer_);
    }

    // Delete History
    if (rtps_history_)
    {
        delete rtps_history_;
    }

    logInfo(DDSROUTER_RTPS_WRITER, "Deleting CommonWriter created in Participant " <<
            participant_id_ << " for topic " << topic_);
}

// Specific enable/disable do not need to be implemented
utils::ReturnCode CommonWriter::write_(
        std::unique_ptr<DataReceived>& data) noexcept
{

    // Take new Change from history
    fastrtps::rtps::CacheChange_t* new_change = rtps_writer_->new_change(eprosima::fastrtps::rtps::ChangeKind_t::ALIVE);

    // If still is not able to get a change, return an error code
    if (!new_change)
    {
        return utils::ReturnCode::RETCODE_ERROR;
    }

    // Get the Payload without copy only if it has length
    if (data->payload.length > 0)
    {
        eprosima::fastrtps::rtps::IPayloadPool* payload_owner = payload_pool_.get();
        if (!payload_pool_->get_payload(data->payload, payload_owner, (*new_change)))
        {
            logDevError(DDSROUTER_RTPS_WRITER, "Error getting Payload.");
            return utils::ReturnCode::RETCODE_ERROR;
        }
    }

    // Set keys in case topic has keys
    if (topic_.keyed)
    {
        new_change->instanceHandle = data->qos.instanceHandle;
    }

    logDebug(DDSROUTER_RTPS_WRITER,
            "CommonWriter " << *this << " sending payload " << new_change->serializedPayload << " from " <<
            data->source_guid);

    if (repeater_)
    {
        // Add origin to change in case the cache change is RouterCacheChange (only in repeater mode)
        types::RouterCacheChange& change_ref = static_cast<types::RouterCacheChange&>(*new_change);
        change_ref.last_writer_guid_prefix = data->source_guid.guidPrefix;
    }

    // Send data by adding it to CommonWriter History
    rtps_history_->add_change(new_change);

    // TODO: check if this being synchronous means that we can remove the change in case of volatile

    // When max history size is reached, remove oldest cache change
    if (rtps_history_->isFull())
    {
        rtps_history_->remove_min_change();
    }

    return utils::ReturnCode::RETCODE_OK;
}

void CommonWriter::internal_entities_creation_(
        const fastrtps::rtps::HistoryAttributes& history_attributes,
        const fastrtps::rtps::WriterAttributes& writer_attributes,
        const fastrtps::TopicAttributes& topic_attributes,
        const fastrtps::WriterQos& writer_qos,
        const utils::PoolConfiguration& pool_configuration)
{
    // Copy writer attributes because fast needs it non const (do not ask why)
    fastrtps::rtps::WriterAttributes non_const_writer_attributes = writer_attributes;

    // Create History
    rtps_history_ = new fastrtps::rtps::WriterHistory(history_attributes);

    // Create CommonWriter
    if (repeater_)
    {
        logDebug(DDSROUTER_RTPS_WRITER, "CommonWriter created with repeater filter");

        rtps_writer_ = fastrtps::rtps::RTPSDomain::createRTPSWriter(
            rtps_participant_,
            non_const_writer_attributes,
            payload_pool_,
            std::make_shared<CacheChangePool>(pool_configuration),
            rtps_history_,
            nullptr);
    }
    else
    {
        rtps_writer_ = fastrtps::rtps::RTPSDomain::createRTPSWriter(
            rtps_participant_,
            non_const_writer_attributes,
            payload_pool_,
            rtps_history_,
            nullptr);
    }

    if (!rtps_writer_)
    {
        throw utils::InitializationException(
                  utils::Formatter() << "Error creating Simple RTPSWriter for Participant " <<
                      participant_id_ << " in topic " << topic_ << ".");
    }

    // Register writer with topic
    if (!rtps_participant_->registerWriter(rtps_writer_, topic_attributes, writer_qos))
    {
        // In case it fails, remove writer and throw exception
        fastrtps::rtps::RTPSDomain::removeRTPSWriter(rtps_writer_);
        throw utils::InitializationException(utils::Formatter() << "Error registering topic " << topic_ <<
                      " for Simple RTPSWriter in Participant " << participant_id_);
    }

    if (repeater_)
    {
        // Use filter writer of origin
        data_filter_ = std::make_unique<RepeaterDataFilter>();
    }
    else
    {
        // Use default filter
        data_filter_ = std::make_unique<SelfDataFilter>();
    }

    rtps_writer_->reader_data_filter(data_filter_.get());

    logInfo(
        DDSROUTER_RTPS_WRITER,
        "New CommonWriter created in Participant " << participant_id_ <<
        " for topic " << topic_ <<
        " with guid " << rtps_writer_->getGuid());
}

fastrtps::rtps::HistoryAttributes CommonWriter::history_attributes_(
        const types::DdsTopic& topic) noexcept
{
    fastrtps::rtps::HistoryAttributes att;

    att.memoryPolicy =
            eprosima::fastrtps::rtps::MemoryManagementPolicy_t::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;

    att.maximumReservedCaches = topic.topic_qos.value.history_depth;
    // TODO: Check if history atts must be set from topics qos

    return att;
}

fastrtps::rtps::WriterAttributes CommonWriter::writer_attributes_(
        const types::DdsTopic& topic) noexcept
{
    fastrtps::rtps::WriterAttributes att;

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

    // TODO Set ownership and partitions

    // Set write mode
    att.mode = fastrtps::rtps::RTPSWriterPublishMode::SYNCHRONOUS_WRITER;

    return att;
}

fastrtps::TopicAttributes CommonWriter::topic_attributes_(
        const types::DdsTopic& topic) noexcept
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

    return att;
}

fastrtps::WriterQos CommonWriter::writer_qos_(
        const types::DdsTopic& topic) noexcept
{
    fastrtps::WriterQos qos;

    // Set Durability
    qos.m_durability.kind =
        (topic.topic_qos.value.is_transient_local()
            ? eprosima::fastdds::dds::DurabilityQosPolicyKind_t::TRANSIENT_LOCAL_DURABILITY_QOS
            : eprosima::fastdds::dds::DurabilityQosPolicyKind_t::VOLATILE_DURABILITY_QOS);

    // Set Reliability
    qos.m_reliability.kind =
        (topic.topic_qos.value.is_reliable()
            ? eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS
            : eprosima::fastdds::dds::ReliabilityQosPolicyKind::BEST_EFFORT_RELIABILITY_QOS);

    // Set minimum deadline so it matches with everything
    qos.m_deadline.period = eprosima::fastrtps::Duration_t(0);

    // TODO Set ownership and partitions

    return qos;
}

utils::PoolConfiguration CommonWriter::cache_change_pool_configuration_(
        const types::DdsTopic& topic) noexcept
{
    utils::PoolConfiguration config;
    config.maximum_size = topic.topic_qos.value.history_depth; // No maximum
    config.initial_size = 20;
    config.batch_size = 20;
    // NOTE: Not use of memory policy or maximum yet

    return config;
}

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
