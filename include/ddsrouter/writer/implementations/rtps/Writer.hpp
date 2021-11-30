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
 * @file Writer.hpp
 */

#ifndef _DDSROUTER_WRITER_IMPLEMENTATIONS_RTPS_WRITER_HPP_
#define _DDSROUTER_WRITER_IMPLEMENTATIONS_RTPS_WRITER_HPP_

#include <fastdds/rtps/rtps_fwd.h>
#include <fastrtps/rtps/attributes/HistoryAttributes.h>
#include <fastrtps/attributes/TopicAttributes.h>
#include <fastrtps/qos/WriterQos.h>
#include <fastrtps/rtps/history/WriterHistory.h>
#include <fastrtps/rtps/attributes/WriterAttributes.h>
#include <fastrtps/rtps/writer/RTPSWriter.h>

#include <ddsrouter/types/participant/ParticipantId.hpp>
#include <ddsrouter/writer/implementations/auxiliar/BaseWriter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace rtps {

/**
 * Standard RTPS Writer with less restrictive Attributes.
 */
class Writer : public BaseWriter
{
public:

    /**
     * @brief Construct a new Writer object
     *
     * Get the Attributes and QoS and create the Writer History and the RTPS Writer.
     *
     * @param participant_id    Router Id of the Participant that has created this Writer.
     * @param topic             Topic that this Writer subscribes to.
     * @param payload_pool      Shared Payload Pool to received data and take it.
     * @param rtps_participant  RTPS Participant pointer (this is not stored).
     *
     * @throw \c InitializationException in case any creation has failed
     */
    Writer(
            const ParticipantId& participant_id,
            const RealTopic& topic,
            std::shared_ptr<PayloadPool> payload_pool,
            fastrtps::rtps::RTPSParticipant* rtps_participant);

    /**
     * @brief Destroy the Writer object
     *
     * Remove Writer RTPS
     * Remove History
     *
     * @todo Remove every change and release it in PayloadPool
     */
    virtual ~Writer();

protected:

    // Specific enable/disable do not need to be implemented

    /**
     * @brief Write specific method
     *
     * Store new data as message to send (asynchronously) (it could use PayloadPool to not copy payload).
     * Take next Untaken Change.
     * Set \c data with the message taken (data payload must be stored from PayloadPool).
     * Remove this change from Reader History and release.
     *
     * It does not require mutex, it will be guarded by RTPS Writer mutex in internal methods.
     *
     * @param data : oldest data to take
     * @return \c RETCODE_OK if data has been correctly taken
     * @return \c RETCODE_NO_DATA if \c data_to_send_ is empty
     * @return \c RETCODE_NO_DATA if \c data_to_send_ is empty
     */
    virtual ReturnCode write_(
            std::unique_ptr<DataReceived>& data) noexcept override;

    /////
    // RTPS specific methods

    /**
     * @brief Default History Attributes to create Writer History
     *
     * @return Default HistoryAttributes
     */
    fastrtps::rtps::HistoryAttributes history_attributes_() const noexcept;

    /**
     * @brief Default Writer Attributes to create Writer
     *
     * It returns the less restrictive Attributes for a Writer, so it maches every Reader
     * durability: TRANSIENT_LOCAL
     * reliability: RELIABLE
     *
     * @warning less restrictive would be PERSISTENCE, but those are not supported yet.
     *
     * @return Default ReaderAttributes
     */
    fastrtps::rtps::WriterAttributes writer_attributes_() const noexcept;

    //! Default Topic Attributes to create Writer
    fastrtps::TopicAttributes topic_attributes_() const noexcept;

    //! Default QoS Writer (must be the same as the attributes)
    fastrtps::WriterQos writer_qos_() const noexcept;

    /////
    // VARIABLES

    //! RTPS Writer pointer
    fastrtps::rtps::RTPSWriter* rtps_writer_;

    //! RTPS Writer History associated to \c rtps_reader_
    fastrtps::rtps::WriterHistory* rtps_history_;
};

} /* namespace rtps */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_WRITER_IMPLEMENTATIONS_RTPS_WRITER_HPP_ */
