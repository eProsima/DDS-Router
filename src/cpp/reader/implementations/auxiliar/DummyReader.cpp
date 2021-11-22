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

#include <ddsrouter/reader/implementations/auxiliar/DummyReader.hpp>

namespace eprosima {
namespace ddsrouter {

DummyReader::DummyReader(
        const ParticipantId& participant_id,
        const RealTopic& topic)
    : participant_id_(participant_id)
    , topic_(topic)
{
}

void DummyReader::enable()
{
    std::lock_guard<std::mutex> lock(mutex_);
    enabled_.store(true);

    // If there is data pending to be sent, call on data available callback
    if (!data_to_send_.empty())
    {
        on_data_available_callback_();
    }
}

void DummyReader::disable()
{
    enabled_.store(false);
}

void DummyReader::set_on_data_available_callback(
        std::function<void()> new_callback)
{
    on_data_available_callback_ = new_callback;
}

ReturnCode DummyReader::take(
        std::unique_ptr<DataReceived>& data_received)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // If it is not enabled, cannot retrieve data
    if (!enabled_.load())
    {
        return ReturnCode::RETCODE_NOT_ENABLED;
    }

    // There is no data pending sent
    if (data_to_send_.empty())
    {
        return ReturnCode::RETCODE_NO_DATA;
    }

    // Get next data received
    DataToSend next_data_to_send = data_to_send_.front();
    data_to_send_.pop();

    // Write (copy) values in data
    data_received->source_guid = next_data_to_send.guid_src;

    // Create space for the data
    data_received->payload.length = next_data_to_send.payload.length;
    data_received->payload.reserve(data_received->payload.length);
    std::memcpy(data_received->payload.data, next_data_to_send.payload.data, data_received->payload.length);

    return ReturnCode::RETCODE_OK;
}

void DummyReader::add_message_to_send(
        DataToSend data)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // Even if disabled, the data will be stored
    data_to_send_.push(data);

    // Call on data available callback
    on_data_available_callback_();
}

} /* namespace ddsrouter */
} /* namespace eprosima */
