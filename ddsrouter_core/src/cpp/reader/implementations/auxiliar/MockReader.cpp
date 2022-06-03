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
 * @file MockReader.cpp
 */

#include <reader/implementations/auxiliar/MockReader.hpp>
#include <ddsrouter_utils/Log.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

using namespace eprosima::ddsrouter::core::types;

void MockReader::simulate_data_reception(
        types::Guid guid,
        void* data,
        uint32_t size) noexcept
{
    std::lock_guard<std::mutex> lock(mock_internal_mutex_);

    std::tuple<types::Guid, void*, uint32_t> data_to_send(guid, data, size);

    data_to_send_.push(std::move(data_to_send));

    on_data_available_();
}

// void MockReader::simulate_data_reception(
//         types::DataReceived&& data) noexcept
// {
//     std::lock_guard<std::mutex> lock(mock_internal_mutex_);

//     // Store data to send it afterwards
//     data_to_send_.push(std::move(data));

//     // Call on data available callback
// }

utils::ReturnCode MockReader::take_(
        std::unique_ptr<DataReceived>& data) noexcept
{
    std::lock_guard<std::mutex> lock(mock_internal_mutex_);

    // There is no data pending sent
    if (data_to_send_.empty())
    {
        return utils::ReturnCode::RETCODE_NO_DATA;
    }

    // Get next data received
    std::tuple<types::Guid, void*, uint32_t>& next_data_to_send = data_to_send_.front();

    // Write (copy) values in data
    data->source_guid = std::get<0>(next_data_to_send);

    // Move Payload to DDSRouter Payload Pool
    // Allocate memory
    payload_pool_->get_payload(std::get<2>(next_data_to_send), data->payload);
    // Copy memory
    std::memcpy(data->payload.data, std::get<1>(next_data_to_send), std::get<2>(next_data_to_send));

    // Remove data completely (it does not deallocate anything as it was only a pointer)
    data_to_send_.pop();

    return utils::ReturnCode::RETCODE_OK;
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
