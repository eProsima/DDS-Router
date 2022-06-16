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

#include <reader/rtps/Reader.hpp>
#include <writer/IWriter.hpp>
#include <communication/DataForwardQueue.hpp>
#include <communication/payload_pool/TopicPayloadPool.hpp>
#include <ddsrouter_core/types/dds/Payload.hpp>
#include <ddsrouter_utils/exception/InitializationException.hpp>
#include <ddsrouter_utils/exception/InconsistencyException.hpp>
#include <ddsrouter_utils/Log.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

using namespace eprosima::ddsrouter::core::types;

void Reader::RTPSReaderDeleter::operator ()(
        fastrtps::rtps::RTPSReader* ptr) const
{
    if (ptr)
    {
        fastrtps::rtps::RTPSDomain::removeRTPSReader(ptr);
    }
}

Reader::Reader(
        const ParticipantId& id,
        const RealTopic& topic,
        std::shared_ptr<fastrtps::rtps::IPayloadPool>& payload_pool,
        DataForwardQueue& data_forward_queue,
        fastrtps::rtps::RTPSParticipant* rtps_participant)
    : IReader(id, topic, payload_pool.get(), data_forward_queue)
    , enqueued_while_disabled_(0)
    , notified_(0)
    , forwarded_(0)
{
    // Set take_lock_ flag to false
    take_lock_.clear();

    // Create History
    rtps_history_ = std::make_unique<fastrtps::rtps::ReaderHistory>(history_attributes_());

    // Create Reader
    fastrtps::rtps::ReaderAttributes reader_att = reader_attributes_();

    rtps_reader_ = std::unique_ptr<fastrtps::rtps::RTPSReader, RTPSReaderDeleter>(
        fastrtps::rtps::RTPSDomain::createRTPSReader(
            rtps_participant,
            reader_att,
            payload_pool,
            rtps_history_.get()),
        RTPSReaderDeleter()
        );

    // Set listener after entity creation to avoid SEGFAULT (produced when callback using rtps_reader_ is
    // invoked before the variable is fully set)
    rtps_reader_->setListener(this);

    if (!rtps_reader_)
    {
        throw utils::InitializationException(
                  utils::Formatter() << "Error creating Simple RTPSReader for Participant " <<
                      id << " in topic " << topic << ".");
    }

    // Register reader with topic
    fastrtps::TopicAttributes topic_att = topic_attributes_();
    fastrtps::ReaderQos reader_qos = reader_qos_();

    if (!rtps_participant->registerReader(rtps_reader_.get(), topic_att, reader_qos))
    {
        throw utils::InitializationException(utils::Formatter() << "Error registering topic " << topic <<
                      " for Simple RTPSReader in Participant " << id);
    }

    logInfo(DDSROUTER_RTPS_READER,
            "New Reader created in Participant " << id_ << " for topic " << topic << " with guid " <<
            rtps_reader_->getGuid());
}

Reader::~Reader()
{
    logInfo(DDSROUTER_RTPS_READER, "Deleting Reader created in Participant " <<
            id_ << " for topic " << topic_);
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

    if (topic_.is_reliable())
    {
        att.endpoint.reliabilityKind = fastrtps::rtps::ReliabilityKind_t::RELIABLE;
        att.endpoint.durabilityKind = fastrtps::rtps::DurabilityKind_t::TRANSIENT_LOCAL;
    }
    else
    {
        att.endpoint.reliabilityKind = fastrtps::rtps::ReliabilityKind_t::BEST_EFFORT;
        att.endpoint.durabilityKind = fastrtps::rtps::DurabilityKind_t::VOLATILE;
    }

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

fastrtps::TopicAttributes Reader::topic_attributes_() const noexcept
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

fastrtps::ReaderQos Reader::reader_qos_() const noexcept
{
    fastrtps::ReaderQos qos;

    if (topic_.is_reliable())
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
        const fastrtps::rtps::CacheChange_t* const in_change) noexcept
{

    logDebug(DDSROUTER_RTPS_READER_LISTENER,
            "Data arrived to Reader " << *this << " with payload " << in_change->serializedPayload << " from ");

    if (in_change->writerGUID.guidPrefix != rtps_reader_->getGuid().guidPrefix)
    {

        if (!this->enabled_.load(std::memory_order_acquire))
        {

            // Topic is disabled

            if (!topic_.is_reliable())
            {
                // Topic not reliable, so remove received change
                rtps_reader_->getHistory()->remove_change((fastrtps::rtps::CacheChange_t*)in_change);
            }
            else
            {
                enqueued_while_disabled_.fetch_add(1, std::memory_order_relaxed);
            }

            return;
        }

        // Change coming from other participant, so push this reader as a task

        data_forward_queue_.push_task( this );

    }
    else
    {
        // If it is a message from this Participant, do not send it forward and remove it
        rtps_reader_->getHistory()->remove_change((fastrtps::rtps::CacheChange_t*)in_change);
    }
}

void Reader::take_and_forward() noexcept
{
    notified_.fetch_add(1, std::memory_order_relaxed);

    // Try to enter to the critical section, or leave
    if (!take_lock_.test_and_set(std::memory_order_acquire))
    {

        // take_lock_ was false, now set to true
        // tacke_lock_.test_and_set with std::memory_order_acquire to make forwarded_ value visible before std::memory_order_release

        while (forwarded_ < notified_.load(std::memory_order_relaxed))
        {

            fastrtps::rtps::CacheChange_t* received_change = nullptr;
            fastrtps::rtps::WriterProxy* wp = nullptr;

            bool nextUntakenCache_success = rtps_reader_->nextUntakenCache(&received_change, &wp);

            if (nextUntakenCache_success && received_change->serializedPayload.max_size > 0 &&
                    received_change->writerGUID != fastrtps::rtps::GUID_t::unknown())
            {

                for (auto writer : writers_)
                {

                    writer->write(received_change);
                }

                rtps_reader_->getHistory()->remove_change(received_change);

                forwarded_++;

            }
            else
            {

                // nextUntakenCache failed or incoming payload has wrong size. Do nothing, just log

                if (!nextUntakenCache_success)
                {

                    logWarning(DDSROUTER_RTPS_READER_LISTENER, "Error taking untaken cache");

                    break;

                }
                else if (received_change->serializedPayload.max_size > 0)
                {

                    rtps_reader_->getHistory()->remove_change(received_change);

                    logWarning(DDSROUTER_RTPS_READER_LISTENER,
                            "Error taking data with length " << received_change->serializedPayload.length << ".");

                    break;

                }
                else   // if (invalid guid)
                {
                    logWarning(DDSROUTER_RTPS_READER_LISTENER, "Unknown writer GUID of received change" << ".");

                    break;
                }

            }
        }

        // std::memory_order_release to make forwarded_ value visible after std::memory_order_acquire
        take_lock_.clear(std::memory_order_release);

    }
    // else take_lock_ was true, left unchanged, and notified_ increased by one in any case
}

void Reader::enable_() noexcept
{
    utils::ReturnCode enqueue_result = utils::ReturnCode::RETCODE_OK;

    while (enqueued_while_disabled_.fetch_sub(1,
            std::memory_order_relaxed) > 0 && enqueue_result != utils::ReturnCode::RETCODE_NO_DATA)
    {
        data_forward_queue_.push_task( this );
    }
}

void Reader::onReaderMatched(
        fastrtps::rtps::RTPSReader*,
        fastrtps::rtps::MatchingInfo& info) noexcept
{
    if (info.remoteEndpointGuid.guidPrefix == rtps_reader_->getGuid().guidPrefix)
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
