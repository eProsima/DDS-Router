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
 * @file QoSSpecificWriter.hpp
 */

#ifndef __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_RTPS_QOSSPECIFICWRITER_HPP_
#define __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_RTPS_QOSSPECIFICWRITER_HPP_

#include <writer/implementations/rtps/CommonWriter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

/**
 * RTPS Writer with Attributes set from QoS.
 * This is a concrete implementation of Writer class where a set of QoS is given
 * and the writer is created with them.
 *
 * If QoS are not set, it uses a default Writer. If they are set, are given in construction.
 */
class QoSSpecificWriter : public CommonWriter
{
public:

    /**
     * @brief Construct a new QoSSpecificWriter object
     *
     * Get the Attributes and TopicQoS and create the QoSSpecificWriter History and the RTPS QoSSpecificWriter.
     *
     * @param participant_id    Router Id of the Participant that has created this QoSSpecificWriter.
     * @param topic             Topic that this QoSSpecificWriter subscribes to.
     * @param payload_pool      Shared Payload Pool to received data and take it.
     * @param rtps_participant  RTPS Participant pointer (this is not stored).
     *
     * @throw \c InitializationException in case any creation has failed
     */
    QoSSpecificWriter(
            const types::ParticipantId& participant_id,
            const types::DdsTopic& topic,
            std::shared_ptr<PayloadPool> payload_pool,
            fastrtps::rtps::RTPSParticipant* rtps_participant,
            const types::SpecificEndpointQoS& specific_qos,
            const bool repeater = false);

protected:

    //! Specific writer QoS to override (more or less) the CommonWriter qos
    static fastrtps::WriterQos writer_qos_(
            const types::SpecificEndpointQoS& specific_qos,
            const types::DdsTopic& topic) noexcept;

    const types::SpecificEndpointQoS& specific_qos_;
};

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_RTPS_QOSSPECIFICWRITER_HPP_ */
