// Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <ddspipe_core/library/library_dll.h>
#include <ddspipe_core/types/dds/Payload.hpp>
#include <ddspipe_core/efficiency/payload/PayloadPool.hpp>
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
struct RtpsPayloadData : public core::IRoutingData
{

    DDSPIPE_CORE_DllAPI RtpsPayloadData() = default;

    /**
     * @brief Destroy the Rtps Payload Data object
     *
     * Free the memory for the payload in the corresponding payload pool (if defined).
     */
    DDSPIPE_CORE_DllAPI virtual ~RtpsPayloadData();

    DDSPIPE_CORE_DllAPI RtpsPayloadData(const RtpsPayloadData& ) = delete;

    DDSPIPE_CORE_DllAPI virtual types::TopicInternalTypeDiscriminator internal_type_discriminator() const noexcept override;

    //! Payload of the data received. The data in this payload must belong to the PayloadPool.
    core::types::Payload payload{};

    /**
     * @brief PayloadPool owner of the payload.
     *
     * If nullptr, the owner has not been set.
     */
    core::PayloadPool* payload_owner{nullptr};

    //! Specific Writer QoS of the Data
    core::types::SpecificEndpointQoS writer_qos{};

    //! Instance of the message (default no instance)
    core::types::InstanceHandle instanceHandle{};

    //! Kind of the change
    core::types::ChangeKind kind{};

    //! Source time stamp of the message
    core::types::DataTime source_timestamp{};

    //! Guid of the source entity that has transmit the data
    core::types::Guid source_guid{};

    //! Id of the participant from which the Reader has received the data.
    core::types::ParticipantId participant_receiver{};
};

/**
 * @brief Id to identify the internal topic type id that uses \c RtpsPayloadData .
 */
const core::types::TopicInternalTypeDiscriminator INTERNAL_TOPIC_TYPE_RTPS = "payload::rtps::v0";

} /* namespace types */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
