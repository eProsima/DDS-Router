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

#include <writer/rtps/Writer.hpp>
#include <reader/IReader.hpp>
#include <ddsrouter_utils/exception/InitializationException.hpp>
#include <ddsrouter_utils/Log.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

using namespace eprosima::ddsrouter::core::types;

void Writer::RTPSWriterDeleter::operator ()(
        fastrtps::rtps::RTPSWriter* ptr) const
{
    if (ptr)
    {
        fastrtps::rtps::RTPSDomain::removeRTPSWriter(ptr);
    }
}

Writer::Writer(
        const ParticipantId& id,
        const RealTopic& topic,
        std::shared_ptr<fastrtps::rtps::IPayloadPool>& payload_pool,
        fastrtps::rtps::RTPSParticipant* rtps_participant)
    : IWriter(id, topic, payload_pool.get())
{
    // Create History
    rtps_history_ = std::make_unique<fastrtps::rtps::WriterHistory>(history_attributes_());

    // Create Writer
    fastrtps::rtps::WriterAttributes writer_att = writer_attributes_();
    rtps_writer_ = std::unique_ptr<fastrtps::rtps::RTPSWriter, RTPSWriterDeleter>(
        fastrtps::rtps::RTPSDomain::createRTPSWriter(
            rtps_participant,
            writer_att,
            payload_pool,
            rtps_history_.get(),
            nullptr),
        RTPSWriterDeleter()
        );

    if (!rtps_writer_)
    {
        throw utils::InitializationException(
                  utils::Formatter() << "Error creating Simple RTPSWriter for Participant " <<
                      id << " in topic " << topic << ".");
    }

    // Register writer with topic
    fastrtps::TopicAttributes topic_att = topic_attributes_();
    fastrtps::WriterQos writer_qos = writer_qos_();

    if (!rtps_participant->registerWriter(rtps_writer_.get(), topic_att, writer_qos))
    {
        throw utils::InitializationException(utils::Formatter() << "Error registering topic " << topic <<
                      " for Simple RTPSWriter in Participant " << id);
    }

    logInfo(DDSROUTER_RTPS_WRITER, "New Writer created in Participant " << id_ << " for topic " <<
            topic << " with guid " << rtps_writer_->getGuid());

}

Writer::~Writer()
{
    logInfo(DDSROUTER_RTPS_WRITER, "Deleting Writer created in Participant " <<
            id_ << " for topic " << topic_);
}

void Writer::write(
        fastrtps::rtps::CacheChange_t* reader_cache_change) noexcept
{
    fastrtps::rtps::CacheChange_t* writer_cache_change = rtps_writer_->new_change(
        eprosima::fastrtps::rtps::ChangeKind_t::ALIVE);

    payload_pool_->get_payload( reader_cache_change->serializedPayload, payload_pool_, *writer_cache_change );

    rtps_history_->add_change(writer_cache_change);
}

fastrtps::rtps::HistoryAttributes Writer::history_attributes_() const noexcept
{
    fastrtps::rtps::HistoryAttributes att;
    att.memoryPolicy =
            eprosima::fastrtps::rtps::MemoryManagementPolicy_t::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
    return att;
}

fastrtps::rtps::WriterAttributes Writer::writer_attributes_() const noexcept
{
    fastrtps::rtps::WriterAttributes att;
    att.endpoint.durabilityKind = eprosima::fastrtps::rtps::DurabilityKind_t::TRANSIENT_LOCAL;
    att.endpoint.reliabilityKind = eprosima::fastrtps::rtps::ReliabilityKind_t::RELIABLE;
    att.mode = fastrtps::rtps::RTPSWriterPublishMode::ASYNCHRONOUS_WRITER;
    if (topic_.has_key())
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
    if (topic_.has_key())
    {
        att.topicKind = eprosima::fastrtps::rtps::WITH_KEY;
    }
    else
    {
        att.topicKind = eprosima::fastrtps::rtps::NO_KEY;
    }
    att.topicName = topic_.name();
    att.topicDataType = topic_.type();
    return att;
}

fastrtps::WriterQos Writer::writer_qos_() const noexcept
{
    fastrtps::WriterQos qos;
    qos.m_durability.kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
    qos.m_reliability.kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    return qos;
}

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
