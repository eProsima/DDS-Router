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
 * @file DummyReader.cpp
 */

#include <fastdds/rtps/common/SerializedPayload.h>
#include <fastdds/rtps/common/CacheChange.h>
#include <reader/auxiliar/DummyReader.hpp>
#include <reader/auxiliar/GenericReader.ipp>
#include <communication/DataForwardQueue.hpp>
#include <writer/IWriter.hpp>
#include <ddsrouter_utils/Log.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

using namespace eprosima::ddsrouter::core::types;

void GenericReader<types::ParticipantKind::dummy>::take_and_forward() noexcept
{
    notified_.fetch_add(1, std::memory_order_relaxed);

    if (not take_lock_.test_and_set(std::memory_order_acquire))
    {

        while (forwarded_ < notified_.load(std::memory_order_relaxed))
        {

            // Get first message in the enqueued
            std::string message;
            {
                std::lock_guard<std::mutex> lck(history_mutex_);
                if (enqueued_messages_.size() > 0)
                {
                    message = enqueued_messages_.front();
                    enqueued_messages_.pop();
                }
            }

            if (message.size() == 0)
            {
                logError(DUMMY_READER, "Message size == 0");
            }

            fastrtps::rtps::SerializedPayload_t serialized_payload(message.size());

            std::unique_ptr<fastrtps::rtps::CacheChange_t> new_change(new fastrtps::rtps::CacheChange_t(message.size()));

            new_change->writerGUID = mock_guid_;

            std::memcpy(new_change->serializedPayload.data, message.c_str(), message.size());

            // Forward to writers
            for (auto writer : writers_)
            {
                writer->write(new_change.get());
            }

            forwarded_++;
        }

        take_lock_.clear(std::memory_order_release);
    }
}

void GenericReader<types::ParticipantKind::dummy>::initialize(
        uint32_t idx)
{
    mock_guid_ = fastrtps::rtps::GUID_t(fastrtps::rtps::GuidPrefix_t(), idx);
    notified_.store(0);
    forwarded_ = 0;
}

void GenericReader<types::ParticipantKind::dummy>::mock_data_reception(
        const std::string& message)
{
    {
        std::lock_guard<std::mutex> lck(history_mutex_);
        enqueued_messages_.push(message);
    }
    data_forward_queue_.push_task(this);
}

unsigned long GenericReader<types::ParticipantKind::dummy>::get_enqueued() const
{
    std::lock_guard<std::mutex> lck(history_mutex_);
    return enqueued_messages_.size();
}

unsigned long GenericReader<types::ParticipantKind::dummy>::get_notified() const
{
    return notified_.load(std::memory_order_seq_cst);
}

unsigned long GenericReader<types::ParticipantKind::dummy>::get_forwarded() const
{
    return forwarded_;
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
