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
 * @file DummyWriter.cpp
 */

#include <writer/implementations/auxiliar/DummyWriter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

using namespace eprosima::ddsrouter::core::types;

utils::ReturnCode DummyWriter::write_(
        std::unique_ptr<DataReceived>& data) noexcept
{
    {
        std::lock_guard<std::mutex> lock(dummy_mutex_);

        // Fill the data to store
        DummyDataStored new_data_to_store;
        new_data_to_store.timestamp = utils::now();
        new_data_to_store.source_guid = data->source_guid;

        // Copying data as it should not be stored in PayloadPool
        for (int i = 0; i < data->payload.length; i++)
        {
            new_data_to_store.payload.push_back(data->payload.data[i]);
        }

        data_stored_.push_back(new_data_to_store);
    }

    // Notify that a new message has been sent
    wait_condition_variable_.notify_all();

    return utils::ReturnCode::RETCODE_OK;
}

void DummyWriter::wait_until_n_data_sent(
        uint16_t n) const noexcept
{
    std::unique_lock<std::mutex> lock(dummy_mutex_);
    wait_condition_variable_.wait(
        lock,
        [n, this]
        {
            return data_stored_.size() >= n;
        });
}

std::vector<DummyDataStored> DummyWriter::get_data_that_should_have_been_sent() const noexcept
{
    std::lock_guard<std::mutex> lock(dummy_mutex_);
    return data_stored_;
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
