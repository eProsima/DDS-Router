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

#include <ddspipe_core/library/library_dll.h>
#include <ddspipe_core/types/data/RtpsPayloadData.hpp>
#include <ddspipe_core/interface/IRoutingData.hpp>
#include <ddspipe_core/types/dds/Guid.hpp>
#include <ddspipe_core/types/dds/SpecificEndpointQoS.hpp>
#include <ddspipe_core/types/participant/ParticipantId.hpp>
#include <ddspipe_core/types/topic/TopicInternalTypeDiscriminator.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {
namespace types {

/**
 * @brief Structure of the Data received from a Reader containing the data itself and its properties.
 *
 * Properties are related information regarding the data and QoS of the source.
 */
struct RpcPayloadData : public RtpsPayloadData
{

    DDSPIPE_CORE_DllAPI virtual types::TopicInternalTypeDiscriminator internal_type_discriminator() const noexcept override;

    //! Write params associated to the received cache change
    utils::Fuzzy<eprosima::fastrtps::rtps::WriteParams> write_params{};

    //! Sequence number of the received cache change
    eprosima::fastrtps::rtps::SequenceNumber_t origin_sequence_number{};

    //! Sequence Number with which the internal writer (ddsrouter writer) has sent this message
    eprosima::fastrtps::rtps::SequenceNumber_t sent_sequence_number;
};

/**
 * @brief Id to identify the internal topic type id that uses \c RpcPayloadData .
 */
const TopicInternalTypeDiscriminator INTERNAL_TOPIC_TYPE_RPC = "type::rpc::v0";

} /* namespace types */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
