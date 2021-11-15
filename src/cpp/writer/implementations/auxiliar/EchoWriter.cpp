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
 * @file EchoWriter.cpp
 */

#include <ddsrouter/writer/implementations/auxiliar/EchoWriter.hpp>

namespace eprosima {
namespace ddsrouter {

EchoWriter::EchoWriter(
        const ParticipantId& participant_id,
        const RealTopic& topic)
    : participant_id_(participant_id)
    , topic_(topic)
    , enabled_(false)
{
}

void EchoWriter::enable()
{
    enabled_.store(true);
}

void EchoWriter::disable()
{
    enabled_.store(false);
}

ReturnCode EchoWriter::write(std::unique_ptr<DataReceived>& data)
{
    if (enabled_.load())
    {
        std::cout << "Echo Participant: " << participant_id_ << " has recived from Endpoint: " << data->source_guid
            << " in topic: " << topic_ << " the following payload: " << data->data << std::endl;

        return ReturnCode::RETCODE_OK;
    }
    else
    {
        return ReturnCode::RETCODE_NOT_ENABLED;
    }
}

} /* namespace ddsrouter */
} /* namespace eprosima */
