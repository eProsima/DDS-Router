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
 * @file PartitionsWriter.hpp
 */

#ifndef __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_RTPS_PARTITIONSWRITER_HPP_
#define __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_RTPS_PARTITIONSWRITER_HPP_

#include <fastdds/rtps/rtps_fwd.h>
#include <fastrtps/rtps/attributes/HistoryAttributes.h>
#include <fastrtps/attributes/TopicAttributes.h>
#include <fastrtps/qos/WriterQos.h>
#include <fastrtps/rtps/history/WriterHistory.h>
#include <fastrtps/rtps/attributes/WriterAttributes.h>
#include <fastrtps/rtps/writer/RTPSWriter.h>

#include <ddsrouter_core/types/participant/ParticipantId.hpp>
#include <writer/implementations/auxiliar/BaseWriter.hpp>
#include <writer/implementations/rtps/QoSSpecificWriter.hpp>
#include <efficiency/cache_change/CacheChangePool.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

/**
 * Standard RTPS PartitionsWriter with less restrictive Attributes.
 *
 * @todo this class could have access to the Discovery DataBase and create writers in discovery and not when
 * data is received.
 */
class PartitionsWriter : public BaseWriter
{
public:
    /**
     * @brief Construct a new PartitionsWriter object
     *
     * Get the Attributes and TopicQoS and create the PartitionsWriter History and the RTPS PartitionsWriter.
     *
     * @param participant_id    Router Id of the Participant that has created this PartitionsWriter.
     * @param topic             Topic that this PartitionsWriter subscribes to.
     * @param payload_pool      Shared Payload Pool to received data and take it.
     * @param rtps_participant  RTPS Participant pointer (this is not stored).
     *
     * @throw \c InitializationException in case any creation has failed
     */
    PartitionsWriter(
        const types::ParticipantId &participant_id,
        const types::DdsTopic &topic,
        std::shared_ptr<PayloadPool> payload_pool,
        fastrtps::rtps::RTPSParticipant *rtps_participant,
        const bool repeater = false);

    /**
     * @brief Destroy the PartitionsWriter object
     *
     * Remove PartitionsWriter RTPS
     * Remove History
     *
     * @todo Remove every change and release it in PayloadPool
     */
    virtual ~PartitionsWriter();

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
     * It does not require mutex, it will be guarded by RTPS PartitionsWriter mutex in internal methods.
     *
     * @param data : oldest data to take
     * @return \c RETCODE_OK if data has been correctly taken
     * @return \c RETCODE_NO_DATA if \c data_to_send_ is empty
     * @return \c RETCODE_NO_DATA if \c data_to_send_ is empty
     */
    virtual utils::ReturnCode write_(
        std::unique_ptr<types::DataReceived> &data) noexcept override;

    bool exist_partition_(const types::DataQoS &data_qos);
    QoSSpecificWriter* get_writer_or_create_(const types::DataQoS &data_qos);
    QoSSpecificWriter* create_writer_nts_(const types::DataQoS &data_qos);

    /////
    // VARIABLES

    // TODO: This could be an unordered_map avoiding the use of operator< with DataQoS,
    // what may be a problem.
    using WritersMapType = utils::SharedAtomicable<std::map<types::DataQoS, QoSSpecificWriter*>>;
    WritersMapType writers_map_;

    fastrtps::rtps::RTPSParticipant* rtps_participant_;
    bool repeater_;
};

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_RTPS_PARTITIONSWRITER_HPP_ */
