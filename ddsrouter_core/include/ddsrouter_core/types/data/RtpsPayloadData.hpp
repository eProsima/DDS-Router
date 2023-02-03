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
 * @file RtpsPayloadData.hpp
 */

#pragma once

#include <fastdds/rtps/common/SerializedPayload.h>
#include <fastdds/rtps/common/SequenceNumber.h>

#include <ddsrouter_core/library/library_dll.h>
#include <ddsrouter_core/types/data/Payload.hpp>
#include <ddsrouter_core/efficiency/payload/PayloadPool.hpp>
#include <ddsrouter_core/types/data/IRoutingData.hpp>
#include <ddsrouter_core/types/dds/Guid.hpp>
#include <ddsrouter_core/types/dds/SpecificEndpointQoS.hpp>
#include <ddsrouter_core/types/participant/ParticipantId.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

/**
 * @brief Structure of the Data received from a Reader containing the data itself and its properties.
 *
 * Properties are related information regarding the data and QoS of the source.
 */
struct RtpsPayloadData : public core::types::IRoutingData
{

    virtual ~RtpsPayloadData();

    //! Payload of the data received. The data in this payload must belong to the PayloadPool.
    Payload payload{};

    core::PayloadPool* payload_owner{nullptr};

    //! Specific Writer QoS of the Data
    core::types::SpecificEndpointQoS writer_qos{};

    //! Instance of the message (default no instance)
    InstanceHandle instanceHandle{};

    //! Kind of the change
    ChangeKind kind{};

    //! Source time stamp of the message
    DataTime source_timestamp{};

    //! Guid of the source entity that has transmit the data
    core::types::Guid source_guid{};

    //! Id of the participant from which the Reader has received the data.
    core::types::ParticipantId participant_receiver{};
};

constexpr const char* RTPS_PAYLOAD_DATA = "rtps::payload@v0";

//! \c octet to stream serializator
DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const RtpsPayloadData& octet);

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
