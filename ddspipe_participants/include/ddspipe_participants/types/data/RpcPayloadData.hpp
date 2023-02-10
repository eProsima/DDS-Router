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


#pragma once

#include <fastdds/rtps/common/SerializedPayload.h>
#include <fastdds/rtps/common/SequenceNumber.h>

#include <cpp_utils/types/Fuzzy.hpp>

#include <ddspipe_participants/library/library_dll.h>
#include <ddspipe_participants/types/data/RtpsPayloadData.hpp>
#include <ddspipe_core/interface/IRoutingData.hpp>
#include <ddspipe_core/types/dds/Guid.hpp>
#include <ddspipe_core/types/dds/SpecificEndpointQoS.hpp>
#include <ddspipe_core/types/participant/ParticipantId.hpp>
#include <ddspipe_core/types/topic/TopicInternalTypeDiscriminator.hpp>

namespace eprosima {
namespace ddspipe {
namespace participants {
namespace types {

/**
 * @brief Structure of the Data received from a Reader containing the data itself and its properties.
 *
 * Properties are related information regarding the data and QoS of the source.
 */
struct RpcPayloadData : public RtpsPayloadData
{
    //! Write params associated to the received cache change
    eprosima::fastrtps::rtps::WriteParams write_params{};

    //! Sequence number of the received cache change
    eprosima::fastrtps::rtps::SequenceNumber_t origin_sequence_number{};
};

/**
 * @brief Id to identify the internal topic type id that uses \c RpcPayloadData .
 */
constexpr const TopicInternalTypeDiscriminator INTERNAL_TOPIC_TYPE_RPC = "type::rpc::v0";

//! \c RpcPayloadData to stream serializator
DDSPIPE_PARTICIPANTS_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const RpcPayloadData& octet);

} /* namespace types */
} /* namespace participants */
} /* namespace ddspipe */
} /* namespace eprosima */
