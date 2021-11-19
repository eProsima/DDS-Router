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

#include <ddsrouter/writer/implementations/auxiliar/DummyWriter.hpp>

namespace eprosima {
namespace ddsrouter {

DummyWriter::DummyWriter(
        const ParticipantId& participant_id,
        const RealTopic& topic)
    : EchoWriter(participant_id, topic)
{
}

ReturnCode DummyWriter::write(std::unique_ptr<DataReceived>& data)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // Fill the data to store
    DataStored new_data_to_store;
    new_data_to_store.timestamp = now();
    new_data_to_store.guid_src = data->source_guid;
    new_data_to_store.payload = data->data;

    data_stored.push_back(new_data_to_store);

    return ReturnCode::RETCODE_OK;
}

std::vector<DataStored> DummyWriter::data_received_ref()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return data_stored;
}


} /* namespace ddsrouter */
} /* namespace eprosima */
