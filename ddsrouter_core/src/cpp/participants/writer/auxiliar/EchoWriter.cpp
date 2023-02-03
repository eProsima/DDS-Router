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

#include <cpp_utils/Log.hpp>

#include <ddsrouter_core/participants/writer/auxiliar/EchoWriter.hpp>
#include <ddsrouter_core/types/data/RtpsPayloadData.hpp>

namespace eprosima {
namespace ddsrouter {
namespace participants {

using namespace eprosima::ddsrouter::core::types;

EchoWriter::EchoWriter(
        const core::types::DdsTopic& topic,
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
        IRoutingData& data) noexcept
{
    auto rtps_data = dynamic_cast<RtpsPayloadData&>(data);
    // TODO: Add Participant receiver Id when added to DataReceived
    if (!verbose_)
    {
        logUser(
            DDSROUTER_ECHO_DATA,
            "Received data in Participant: " << rtps_data.properties.participant_receiver <<
                " in topic: " << topic_ <<
                ".");
    }
    else
    {
        logUser(
            DDSROUTER_ECHO_DATA,
            "In Endpoint: " << rtps_data.properties.source_guid <<
                " from Participant: " << rtps_data.properties.participant_receiver <<
                " in topic: " << topic_ <<
                " payload received: " << rtps_data.payload <<
                " with specific qos: " << rtps_data.properties.writer_qos <<
                ".");
    }

    return utils::ReturnCode::RETCODE_OK;
}

} /* namespace participants */
} /* namespace ddsrouter */
} /* namespace eprosima */
