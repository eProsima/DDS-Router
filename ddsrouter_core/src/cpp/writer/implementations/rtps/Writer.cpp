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
 * @file Writer.cpp
 */

#include <fastrtps/rtps/RTPSDomain.h>
#include <fastrtps/rtps/participant/RTPSParticipant.h>
#include <fastrtps/rtps/common/CacheChange.h>

#include <writer/implementations/rtps/Writer.hpp>

#include <ddsrouter_core/types/topic/RPCTopic.hpp>
#include <ddsrouter_utils/exception/InitializationException.hpp>
#include <ddsrouter_utils/Log.hpp>
#include <efficiency/cache_change/CacheChangePool.hpp>
#include <writer/implementations/rtps/filter/RepeaterDataFilter.hpp>
#include <writer/implementations/rtps/filter/SelfDataFilter.hpp>
#include <types/dds/RouterCacheChange.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

using namespace eprosima::ddsrouter::core::types;

Writer::Writer(
        const ParticipantId& participant_id,
        const RealTopic& topic,
        std::shared_ptr<PayloadPool> payload_pool,
        fastrtps::rtps::RTPSParticipant* rtps_participant,
        unsigned int max_history_depth,
        const bool repeater /* = false */)
    : BaseWriter(participant_id, topic, payload_pool)
    , repeater_(repeater)
    , write_with_params_(false)
{
    // TODO Use payload pool for this writer, so change does not need to be copied

    // Create History
    fastrtps::rtps::HistoryAttributes history_att = history_attributes_();
    history_att.maximumReservedCaches = max_history_depth;
    rtps_history_ = new fastrtps::rtps::WriterHistory(history_att);

    // Create Writer
    fastrtps::rtps::WriterAttributes writer_att = writer_attributes_();

    if (repeater)
    {
        logDebug(DDSROUTER_RTPS_WRITER, "Writer created with repeater filter");

        utils::PoolConfiguration pool_config = cache_change_pool_configuration_();
        rtps_writer_ = fastrtps::rtps::RTPSDomain::createRTPSWriter(
            rtps_participant,
            writer_att,
            payload_pool_,
            std::make_shared<CacheChangePool>(pool_config),
            rtps_history_,
            nullptr);
    }
    else
    {
        rtps_writer_ = fastrtps::rtps::RTPSDomain::createRTPSWriter(
            rtps_participant,
            writer_att,
            payload_pool_,
            rtps_history_,
            nullptr);
    }

    if (!rtps_writer_)
    {
        throw utils::InitializationException(
                  utils::Formatter() << "Error creating Simple RTPSWriter for Participant " <<
                      participant_id << " in topic " << topic_ << ".");
    }

    // Register writer with topic
    fastrtps::TopicAttributes topic_att = topic_attributes_();
    fastrtps::WriterQos writer_qos = writer_qos_();

    if (!rtps_participant->registerWriter(rtps_writer_, topic_att, writer_qos))
    {
        // In case it fails, remove writer and throw exception
        fastrtps::rtps::RTPSDomain::removeRTPSWriter(rtps_writer_);
        throw utils::InitializationException(utils::Formatter() << "Error registering topic " << topic_ <<
                      " for Simple RTPSWriter in Participant " << participant_id);
    }

    if (repeater)
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

    logInfo(DDSROUTER_RTPS_WRITER, "New Writer created in Participant " << participant_id_ << " for topic " <<
            topic_ << " with guid " << rtps_writer_->getGuid());
}

Writer::~Writer()
{
    // This variables should be set, otherwise the creation should have fail
    // Anyway, the if case is used for safety reasons

    // Delete writer
    if (rtps_writer_)
    {
        // Delete the Writer the History is cleaned
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

void Writer::onWriterMatched(
        fastrtps::rtps::RTPSWriter* writer,
        fastrtps::rtps::MatchingInfo& info)
{
    if (!come_from_this_participant_(info.remoteEndpointGuid))
    {
        if (info.status == fastrtps::rtps::MATCHED_MATCHING)
        {
            logInfo(DDSROUTER_RTPS_WRITER, "Writer matched in Participant " << participant_id_ << " for topic " <<
                    topic_ << " with guid " << writer->getGuid() << " matched with " << info.remoteEndpointGuid);
        }
        else
        {
            logInfo(DDSROUTER_RTPS_WRITER, "Writer unmatched in Participant " << participant_id_ << " for topic " <<
                    topic_ << " with guid " << writer->getGuid() << " matched with " << info.remoteEndpointGuid);
        }
    }
}

utils::ReturnCode Writer::write(
        std::unique_ptr<DataReceived>& data,
        WriteParams& wparams,
        SequenceNumber& sequenceNumber) noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (enabled_.load())
    {
        // Set flag and copy parameters to object attribute
        write_with_params_ = true;
        write_info_.write_params = wparams;

        utils::ReturnCode ret = write_(data);

        // Copy write params and sequence number to given references
        wparams = write_info_.write_params;
        sequenceNumber = write_info_.sequence_number;

        // Write operation finished -> set flag to false
        write_with_params_ = false;
        return ret;
    }
    else
    {
        logDevError(DDSROUTER_RTPS_WRITER, "Attempt to write data from disabled Writer in topic " <<
                topic_ << " in Participant " << participant_id_);
        return utils::ReturnCode::RETCODE_NOT_ENABLED;
    }
}

utils::ReturnCode Writer::write(
        std::unique_ptr<DataReceived>& data,
        WriteParams& wparams) noexcept
{
    SequenceNumber _dummy;
    return write(data, wparams, _dummy);
}

// Specific enable/disable do not need to be implemented
utils::ReturnCode Writer::write_(
        std::unique_ptr<DataReceived>& data) noexcept
{

    // Take new Change from history
    fastrtps::rtps::CacheChange_t* new_change = rtps_writer_->new_change(eprosima::fastrtps::rtps::ChangeKind_t::ALIVE);

    // TODO : Set method to remove old changes in order to get a new one
    // In case it fails, remove old changes from history and try again
    // uint32_t data_size = data->payload.length;
    // if (!new_change)
    // {
    //     rtps_writer_->remove_older_changes(1);
    //     new_change = rtps_writer_->new_change([data_size]() -> uint32_t
    //     {
    //         return data_size;
    //     }, eprosima::fastrtps::rtps::ChangeKind_t::ALIVE);
    // }

    // If still is not able to get a change, return an error code
    if (!new_change)
    {
        return utils::ReturnCode::RETCODE_ERROR;
    }

    // Get the Payload (copying it)
    eprosima::fastrtps::rtps::IPayloadPool* payload_owner = payload_pool_.get();
    if (!payload_pool_->get_payload(data->payload, payload_owner, (*new_change)))
    {
        logDevError(DDSROUTER_RTPS_WRITER, "Error getting Payload.");
        return utils::ReturnCode::RETCODE_ERROR;
    }

    logDebug(DDSROUTER_RTPS_WRITER,
            "Writer " << *this << " sending payload " << new_change->serializedPayload << " from " <<
            data->source_guid);

    if (repeater_)
    {
        // Add origin to change in case the cache change is RouterCacheChange (only in repeater mode)
        types::RouterCacheChange& change_ref = static_cast<types::RouterCacheChange&>(*new_change);
        change_ref.last_writer_guid_prefix = data->source_guid.guidPrefix;
    }

    // Send data by adding it to Writer History
    if (write_with_params_)
    {
        rtps_history_->add_change(new_change, write_info_.write_params);
    }
    else
    {
        rtps_history_->add_change(new_change);
    }

    // Copy sequence number of write operation to object attribute
    write_info_.sequence_number = new_change->sequenceNumber;

    // When max history size is reached, remove oldest cache change
    if (rtps_history_->isFull())
    {
        rtps_history_->remove_min_change();
    }

    return utils::ReturnCode::RETCODE_OK;
}

bool Writer::come_from_this_participant_(
        const fastrtps::rtps::GUID_t guid) const noexcept
{
    return guid.guidPrefix == rtps_writer_->getGuid().guidPrefix;
}

fastrtps::rtps::HistoryAttributes Writer::history_attributes_() const noexcept
{
    fastrtps::rtps::HistoryAttributes att;
    att.memoryPolicy =
            eprosima::fastrtps::rtps::MemoryManagementPolicy_t::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
    return att;
}

fastrtps::rtps::WriterAttributes Writer::writer_attributes_() noexcept
{
    fastrtps::rtps::WriterAttributes att;

    // TMP: until Transparency module is available
    if (RPCTopic::is_service_topic(topic_))
    {
        // Default ROS 2 service QoS (custom QoS not supported until transparency module is available)
        att.endpoint.durabilityKind = eprosima::fastrtps::rtps::DurabilityKind_t::VOLATILE;
        att.endpoint.reliabilityKind = eprosima::fastrtps::rtps::ReliabilityKind_t::RELIABLE;

        topic_.topic_reliable(true);
    }
    else
    {
        att.endpoint.durabilityKind = eprosima::fastrtps::rtps::DurabilityKind_t::TRANSIENT_LOCAL;
        att.endpoint.reliabilityKind = eprosima::fastrtps::rtps::ReliabilityKind_t::RELIABLE;
    }
    // Write synchronously to avoid removing a change before being sent, likely event when history depth is very small
    att.mode = fastrtps::rtps::RTPSWriterPublishMode::SYNCHRONOUS_WRITER;
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

fastrtps::TopicAttributes Writer::topic_attributes_() const noexcept
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

fastrtps::WriterQos Writer::writer_qos_() noexcept
{
    fastrtps::WriterQos qos;

    // TMP: until Transparency module is available
    if (RPCTopic::is_service_topic(topic_))
    {
        // Default ROS 2 service QoS (custom QoS not supported until transparency module is available)
        qos.m_durability.kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::VOLATILE_DURABILITY_QOS;
        qos.m_reliability.kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;

        topic_.topic_reliable(true);
    }
    else
    {
        qos.m_durability.kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
        qos.m_reliability.kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    }
    return qos;
}

utils::PoolConfiguration Writer::cache_change_pool_configuration_() const noexcept
{
    utils::PoolConfiguration config;
    config.maximum_size = 0; // No maximum
    config.initial_size = 20;
    config.batch_size = 20;
    // NOTE: Not use of memory policy or maximum yet

    return config;
}

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
