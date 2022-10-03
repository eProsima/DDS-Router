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
 * @file SimpleWriter.cpp
 */

#include <fastrtps/rtps/RTPSDomain.h>
#include <fastrtps/rtps/participant/RTPSParticipant.h>
#include <fastrtps/rtps/common/CacheChange.h>

#include <cpp_utils/exception/InitializationException.hpp>
#include <cpp_utils/Log.hpp>
#include <efficiency/cache_change/CacheChangePool.hpp>
#include <writer/implementations/rtps/SimpleWriter.hpp>
#include <writer/implementations/rtps/filter/RepeaterDataFilter.hpp>
#include <writer/implementations/rtps/filter/SelfDataFilter.hpp>
#include <types/dds/RouterCacheChange.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

using namespace eprosima::ddsrouter::core::types;

SimpleWriter::SimpleWriter(
        const ParticipantId& participant_id,
        const DdsTopic& topic,
        std::shared_ptr<PayloadPool> payload_pool,
        fastrtps::rtps::RTPSParticipant* rtps_participant,
        const bool repeater /* = false */)
    : CommonWriter(
        participant_id, topic, payload_pool, rtps_participant, repeater,
        history_attributes_(topic),
        writer_attributes_(topic),
        topic_attributes_(topic),
        writer_qos_(topic),
        cache_change_pool_configuration_(topic))
{
}

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
