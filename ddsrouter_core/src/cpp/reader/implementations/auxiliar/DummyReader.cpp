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

#include <reader/implementations/auxiliar/DummyReader.hpp>
#include <ddsrouter_utils/Log.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

using namespace eprosima::ddsrouter::core::types;

void DummyReader::simulate_data_reception(
        DummyDataReceived data) noexcept
{
    std::lock_guard<std::recursive_mutex> lock(dummy_mutex_);

    // Even if disabled, the data will be stored
    data_to_send_.push(data);

    // Call on data available callback
    on_data_available_();
}

utils::ReturnCode DummyReader::take_(
        fastrtps::rtps::CacheChange_t*& cache_change) noexcept
{
    std::lock_guard<std::recursive_mutex> lock(dummy_mutex_);

    // Enable check is done in BaseReader

    // There is no data pending sent
    if (data_to_send_.empty())
    {
        return utils::ReturnCode::RETCODE_NO_DATA;
    }

    // TMP TODO anton, once interfaces are clarified
    // // Get next data received
    // DummyDataReceived next_data_to_send = data_to_send_.front();
    // data_to_send_.pop();

    // // Write (copy) values in data
    // data->source_guid = next_data_to_send.source_guid;

    // Move Payload to DDSRouter Payload Pool
    payload_pool_->get_payload(
        static_cast<uint32_t>(next_data_to_send.payload.size() * sizeof(PayloadUnit)),
        data->payload);

    // Set values in Payload as the data was not in the DDSRouter Payload Pool
    for (int i = 0; i < next_data_to_send.payload.size(); i++)
    {
        data->payload.data[i] = next_data_to_send.payload[i];
    }
    data->payload.length = data->payload.max_size;

    return utils::ReturnCode::RETCODE_OK;
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
