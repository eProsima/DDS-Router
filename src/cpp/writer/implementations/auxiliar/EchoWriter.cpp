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

ReturnCode EchoWriter::write_(
        std::unique_ptr<DataReceived>& data) noexcept
{
    std::cout << "Echo Participant: " << participant_id_ << " has received from Endpoint: " << data->source_guid
                << " in topic: " << topic_ << " the following payload: <" << data->payload << ">" << std::endl;

    return ReturnCode::RETCODE_OK;
}

} /* namespace ddsrouter */
} /* namespace eprosima */
