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
 * @file MockWriter.cpp
 */

#include <writer/implementations/auxiliar/MockWriter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

using namespace eprosima::ddsrouter::core::types;

MockWriter::MockWriter(
        const types::ParticipantId& participant_id,
        const types::RealTopic& topic,
        std::shared_ptr<PayloadPool> payload_pool)
    : BaseWriter(participant_id, topic, payload_pool)
    , counter_wait_handler_(0)
    , callback_([](std::unique_ptr<types::DataReceived>&){})
{
}

void MockWriter::register_write_callback(
    const std::function<void(std::unique_ptr<types::DataReceived>&)>& callback)
{
    callback_ = callback;
}

utils::ReturnCode MockWriter::write_(
        std::unique_ptr<DataReceived>& data) noexcept
{
    // Simulate send data by callback
    callback_(data);

    // Notify that a new message has been sent
    ++counter_wait_handler_;

    return utils::ReturnCode::RETCODE_OK;
}

void MockWriter::wait_until_n_data_sent(
        uint32_t n) noexcept
{
    counter_wait_handler_.wait_greater_equal_than(n);
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
