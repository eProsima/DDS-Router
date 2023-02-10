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
#include <ddspipe_core/efficiency/cache_change/CacheChangePool.hpp>
#include <ddspipe_participants/writer/rtps/SimpleWriter.hpp>
#include <ddspipe_participants/writer/rtps/filter/RepeaterDataFilter.hpp>
#include <ddspipe_participants/writer/rtps/filter/SelfDataFilter.hpp>
#include <types/dds/RouterCacheChange.hpp>

namespace eprosima {
namespace ddspipe {
namespace participants {
namespace rtps {

using namespace eprosima::ddsrouter::core::types;

SimpleWriter::SimpleWriter(
        const ParticipantId& participant_id,
        const DistributedTopic& topic,
        std::shared_ptr<core::PayloadPool> payload_pool,
        fastrtps::rtps::RTPSParticipant* rtps_participant,
        const bool repeater /* = false */)
    : CommonWriter(
        participant_id, topic, payload_pool, rtps_participant, repeater,
        get_history_attributes_(topic),
        get_writer_attributes_(topic),
        get_topic_attributes_(topic),
        get_writer_qos_(topic),
        cache_change_pool_configuration_(topic))
{
}

} /* namespace rtps */
} /* namespace participants */
} /* namespace ddspipe */
} /* namespace eprosima */
