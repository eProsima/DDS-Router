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

#include <cpp_utils/types/Atomicable.hpp>

#include <ddspipe_core/types/participant/ParticipantId.hpp>
#include <ddspipe_core/types/topic/dds/DdsTopic.hpp>

#include <ddspipe_participants/library/library_dll.h>
#include <ddspipe_participants/writer/auxiliar/BaseWriter.hpp>
#include <ddspipe_participants/writer/rtps/QoSSpecificWriter.hpp>

namespace eprosima {
namespace ddspipe {
namespace participants {
namespace rtps {

/**
 * Writer collection implementation that contains multiple QoSSpecificWriter.
 *
 * @todo this class could have access to the Discovery DataBase and create writers in discovery and not when
 * data is received.
 */
class MultiWriter : public BaseWriter
{
public:

    /**
     * @brief Construct a new MultiWriter object
     *
     * Get the Attributes and TopicQoS and create the MultiWriter History and the RTPS MultiWriter.
     *
     * @param participant_id    Router Id of the Participant that has created this MultiWriter.
     * @param topic             Topic that this MultiWriter subscribes to.
     * @param payload_pool      Shared Payload Pool to received data and take it.
     * @param rtps_participant  RTPS Participant pointer (this is not stored).
     *
     * @throw \c InitializationException in case any creation has failed
     */
    DDSPIPE_PARTICIPANTS_DllAPI MultiWriter(
            const core::types::ParticipantId& participant_id,
            const core::types::DdsTopic& topic,
            const std::shared_ptr<core::PayloadPool>& payload_pool,
            fastrtps::rtps::RTPSParticipant* rtps_participant,
            const bool repeater = false);

    /**
     * @brief Destroy the MultiWriter object
     *
     * Remove MultiWriter RTPS
     * Remove History
     *
     * @todo Remove every change and release it in PayloadPool
     */
    DDSPIPE_PARTICIPANTS_DllAPI virtual ~MultiWriter();

protected:

    //! Override specific enable to call enable in internal writers.
    virtual void enable_() noexcept override;
    //! Override specific disable to call disable in internal writers.
    virtual void disable_() noexcept override;

    /**
     * TODO
     */
    virtual utils::ReturnCode write_(
            core::IRoutingData& data) noexcept override;

    bool exist_partition_(
            const core::types::SpecificEndpointQoS& data_qos);
    QoSSpecificWriter* get_writer_or_create_(
            const core::types::SpecificEndpointQoS& data_qos);
    QoSSpecificWriter* create_writer_nts_(
            const core::types::SpecificEndpointQoS& data_qos);

    /////////////////////////
    // INTERNAL VARIABLES
    /////////////////////////

    using WritersMapType = utils::SharedAtomicable<std::map<core::types::SpecificEndpointQoS, QoSSpecificWriter*>>;
    //! Map of writer indexed by Specific QoS of each.
    WritersMapType writers_map_;

    core::types::DdsTopic topic_;

    //! Reference to RTPS Participant.
    fastrtps::rtps::RTPSParticipant* rtps_participant_;

    //! Whether this Writer is a repeater.
    bool repeater_;
};

} /* namespace rtps */
} /* namespace participants */
} /* namespace ddspipe */
} /* namespace eprosima */
