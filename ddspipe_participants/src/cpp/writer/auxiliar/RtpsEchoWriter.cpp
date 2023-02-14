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

#include <cpp_utils/Log.hpp>

#include <ddspipe_core/types/data/RtpsPayloadData.hpp>

#include <ddspipe_participants/writer/auxiliar/RtpsEchoWriter.hpp>

namespace eprosima {
namespace ddspipe {
namespace participants {

using eprosima::ddspipe::core::types::operator<<;

RtpsEchoWriter::RtpsEchoWriter(
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

utils::ReturnCode RtpsEchoWriter::write(
        core::IRoutingData& data) noexcept
{
    auto& rtps_data = dynamic_cast<core::types::RtpsPayloadData&>(data);

    // TODO: Add Participant receiver Id when added to DataReceived
    if (!verbose_)
    {
        logUser(
            DDSROUTER_ECHO_DATA,
            "Received data in Participant: " << rtps_data.participant_receiver <<
                " in topic: " << topic_.topic_name() <<
                ".");
    }
    else
    {
        logUser(
            DDSROUTER_ECHO_DATA,
            "In Endpoint: " << rtps_data.source_guid <<
                " from Participant: " << rtps_data.participant_receiver <<
                " in topic: " << topic_.topic_name() <<
                " payload received: " << rtps_data.payload <<
                " with specific qos: " << rtps_data.writer_qos <<
                ".");
    }

    return utils::ReturnCode::RETCODE_OK;
}

} /* namespace participants */
} /* namespace ddspipe */
} /* namespace eprosima */
