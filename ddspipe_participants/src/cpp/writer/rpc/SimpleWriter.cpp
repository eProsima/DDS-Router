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
#include <fastrtps/rtps/common/CacheChange.h>

#include <cpp_utils/exception/InitializationException.hpp>
#include <cpp_utils/Log.hpp>

#include <ddspipe_core/types/data/RpcPayloadData.hpp>

#include <ddspipe_participants/efficiency/cache_change/CacheChangePool.hpp>
#include <ddspipe_participants/writer/rpc/SimpleWriter.hpp>
#include <ddspipe_participants/writer/rtps/filter/RepeaterDataFilter.hpp>
#include <ddspipe_participants/writer/rtps/filter/SelfDataFilter.hpp>
#include <ddspipe_participants/types/dds/RouterCacheChange.hpp>

namespace eprosima {
namespace ddspipe {
namespace participants {
namespace rpc {

SimpleWriter::SimpleWriter(
        const core::types::ParticipantId& participant_id,
        const core::types::DdsTopic& topic,
        const std::shared_ptr<core::PayloadPool>& payload_pool,
        fastrtps::rtps::RTPSParticipant* rtps_participant,
        const bool repeater /* = false */)
    : CommonWriter(
        participant_id, topic, payload_pool, rtps_participant, repeater,
        reckon_history_attributes_(topic),
        reckon_writer_attributes_(topic),
        reckon_topic_attributes_(topic),
        reckon_writer_qos_(topic),
        reckon_cache_change_pool_configuration_(topic))
{
    logInfo(DDSROUTER_RPC_WRITER, "Creating RPC Writer for topic " << topic_);
}

utils::ReturnCode SimpleWriter::fill_to_send_data_(
            fastrtps::rtps::CacheChange_t* to_send_change_to_fill,
            eprosima::fastrtps::rtps::WriteParams& to_send_params,
            const core::types::RtpsPayloadData& data) const noexcept
{
    CommonWriter::fill_to_send_data_(
        to_send_change_to_fill,
        to_send_params,
        data);

    const core::types::RpcPayloadData& rpc_data = dynamic_cast<const core::types::RpcPayloadData&>(data);
    if (rpc_data.write_params.is_set())
    {
        to_send_params.related_sample_identity(rpc_data.write_params.get_reference().related_sample_identity());
    }

    return utils::ReturnCode::RETCODE_OK;
}

void SimpleWriter::fill_sent_data_(
        const eprosima::fastrtps::rtps::WriteParams& sent_params,
        core::types::RtpsPayloadData& data_to_fill) const noexcept
{
    CommonWriter::fill_sent_data_(
        sent_params,
        data_to_fill);

    core::types::RpcPayloadData& rpc_data = dynamic_cast<core::types::RpcPayloadData&>(data_to_fill);
    rpc_data.sent_sequence_number = sent_params.sample_identity().sequence_number();
}

} /* namespace rpc */
} /* namespace participants */
} /* namespace ddspipe */
} /* namespace eprosima */
