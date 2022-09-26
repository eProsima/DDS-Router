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
 * @file QoSSpecificWriter.cpp
 */

#include <writer/implementations/rtps/QoSSpecificWriter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

using namespace eprosima::ddsrouter::core::types;

QoSSpecificWriter::QoSSpecificWriter(
        const ParticipantId& participant_id,
        const DdsTopic& topic,
        std::shared_ptr<PayloadPool> payload_pool,
        fastrtps::rtps::RTPSParticipant* rtps_participant,
        const types::SpecificEndpointQoS& specific_qos,
        const bool repeater /* = false */)
    : CommonWriter(
        participant_id, topic, payload_pool, rtps_participant, repeater,
        history_attributes_(topic),
        writer_attributes_(topic),
        topic_attributes_(topic),
        writer_qos_(specific_qos, topic),  // this modifies the qos of the Common Writer
        cache_change_pool_configuration_(topic))
    , specific_qos_(specific_qos)
{
}

fastrtps::WriterQos QoSSpecificWriter::writer_qos_(
        const types::SpecificEndpointQoS& specific_qos,
        const types::DdsTopic& topic) noexcept
{
    // Get QoS from parent class
    fastrtps::WriterQos qos = CommonWriter::writer_qos_(topic);

    // Set Partitions
    if (topic.topic_qos.value.has_partitions())
    {
        qos.m_partition = specific_qos.partitions;
    }

    // Set Ownership
    if (topic.topic_qos.value.has_ownership())
    {
        // Set ownership
        qos.m_ownership.kind = topic.topic_qos.value.ownership_qos;

        qos.m_ownershipStrength = specific_qos.ownership_strength;
    }

    return qos;
}

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
