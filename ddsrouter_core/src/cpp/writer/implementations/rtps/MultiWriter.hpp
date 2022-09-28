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
 * @file MultiWriter.hpp
 */

#ifndef __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_RTPS_PARTITIONSWRITER_HPP_
#define __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_RTPS_PARTITIONSWRITER_HPP_

#include <ddsrouter_utils/types/Atomicable.hpp>

#include <ddsrouter_core/types/participant/ParticipantId.hpp>
#include <writer/implementations/auxiliar/BaseWriter.hpp>
#include <writer/implementations/rtps/QoSSpecificWriter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
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
    MultiWriter(
        const types::ParticipantId &participant_id,
        const types::DdsTopic &topic,
        std::shared_ptr<PayloadPool> payload_pool,
        fastrtps::rtps::RTPSParticipant *rtps_participant,
        const bool repeater = false);

    /**
     * @brief Destroy the MultiWriter object
     *
     * Remove MultiWriter RTPS
     * Remove History
     *
     * @todo Remove every change and release it in PayloadPool
     */
    virtual ~MultiWriter();

protected:
    // Specific enable/disable.
    virtual void enable_() noexcept override;
    virtual void disable_() noexcept override;

    /**
     * @brief Write specific method
     *
     * Store new data as message to send (asynchronously) (it could use PayloadPool to not copy payload).
     * Take next Untaken Change.
     * Set \c data with the message taken (data payload must be stored from PayloadPool).
     * Remove this change from Reader History and release.
     *
     * It does not require mutex, it will be guarded by RTPS MultiWriter mutex in internal methods.
     *
     * @param data : oldest data to take
     * @return \c RETCODE_OK if data has been correctly taken
     * @return \c RETCODE_NO_DATA if \c data_to_send_ is empty
     * @return \c RETCODE_NO_DATA if \c data_to_send_ is empty
     */
    virtual utils::ReturnCode write_(
        std::unique_ptr<types::DataReceived> &data) noexcept override;

    bool exist_partition_(const types::SpecificEndpointQoS &data_qos);
    QoSSpecificWriter* get_writer_or_create_(const types::SpecificEndpointQoS &data_qos);
    QoSSpecificWriter* create_writer_nts_(const types::SpecificEndpointQoS &data_qos);

    /////
    // VARIABLES

    // TODO: This could be an unordered_map avoiding the use of operator< with SpecificEndpointQoS,
    // what may be a problem.
    using WritersMapType = utils::SharedAtomicable<std::map<types::SpecificEndpointQoS, QoSSpecificWriter*>>;
    //! Map of writer indexed by Specific QoS of each.
    WritersMapType writers_map_;

    //! Reference to RTPS Participant.
    fastrtps::rtps::RTPSParticipant* rtps_participant_;

    //! Whether this Writer is a repeater.
    bool repeater_;
};

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_RTPS_PARTITIONSWRITER_HPP_ */
