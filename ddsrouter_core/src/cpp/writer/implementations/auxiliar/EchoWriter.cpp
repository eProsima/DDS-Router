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

#include <ddsrouter_utils/Log.hpp>

#include <writer/implementations/auxiliar/EchoWriter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

using namespace eprosima::ddsrouter::core::types;

EchoWriter::EchoWriter(
        const types::RealTopic& topic,
        bool verbose)
    : topic_(topic)
    , verbose_(verbose)
{
    logDebug(
        DDSROUTER_BASEWRITER,
        "Creating Echo Writer with verbose: " <<
        (verbose_ ? "active" : "inactive") << ".");
}

utils::ReturnCode EchoWriter::write(
        std::unique_ptr<DataReceived>& data) noexcept
{
    // TODO: Add Participant receiver Id when added to DataReceived
    if (!verbose_)
    {
        logUser(
            DDSROUTER_ECHO_DATA,
            "Received data in Participant: " << data->participant_receiver <<
            " in topic: " << topic_ <<
            ".");
    }
    else
    {
        logUser(
            DDSROUTER_ECHO_DATA,
            "In Endpoint: " << data->source_guid <<
            " from Participant: " << data->participant_receiver <<
            " in topic: " << topic_ <<
            " payload received: " << data->payload <<
            ".");
    }

    return utils::ReturnCode::RETCODE_OK;
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
