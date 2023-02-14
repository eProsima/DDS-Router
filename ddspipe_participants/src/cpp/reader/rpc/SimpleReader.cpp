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

#include <fastrtps/rtps/RTPSDomain.h>
#include <fastrtps/rtps/participant/RTPSParticipant.h>

#include <cpp_utils/exception/InitializationException.hpp>
#include <cpp_utils/Log.hpp>

#include <ddspipe_core/types/data/RpcPayloadData.hpp>

#include <ddspipe_participants/reader/rpc/SimpleReader.hpp>

namespace eprosima {
namespace ddspipe {
namespace participants {
namespace rpc {

SimpleReader::SimpleReader(
        const core::types::ParticipantId& participant_id,
        const core::types::DdsTopic& topic,
        const std::shared_ptr<core::PayloadPool>& payload_pool,
        fastrtps::rtps::RTPSParticipant* rtps_participant)
    : CommonReader(
        participant_id, topic, payload_pool, rtps_participant,
        reckon_history_attributes_(topic),
        reckon_reader_attributes_(topic),
        reckon_topic_attributes_(topic),
        reckon_reader_qos_(topic))
{
    logInfo(DDSROUTER_RPC_READER, "Creating RPC Reader for topic " << topic_);
}

//! Override Parent method to create an RPC data type.
core::types::RtpsPayloadData* SimpleReader::create_data_(
        const fastrtps::rtps::CacheChange_t& received_change) const noexcept
{
    return new core::types::RpcPayloadData();
}

//! Override Parent method to fill fields exclusive from RPC.
void SimpleReader::fill_received_data_(
        const fastrtps::rtps::CacheChange_t& received_change,
        core::types::RtpsPayloadData& data_to_fill) const noexcept
{
    CommonReader::fill_received_data_(received_change, data_to_fill);

    // Get internal RpcPayload
    core::types::RpcPayloadData& rpc_data = dynamic_cast<core::types::RpcPayloadData&>(data_to_fill);
    // Set write params and origin sequence number
    rpc_data.write_params.set_value(received_change.write_params);
    rpc_data.origin_sequence_number = received_change.sequenceNumber;
}

} /* namespace rpc */
} /* namespace participants */
} /* namespace ddspipe */
} /* namespace eprosima */
