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

#ifndef __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_RTPS_WRITER_HPP_
#define __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_RTPS_WRITER_HPP_

#include <fastdds/rtps/rtps_fwd.h>
#include <fastrtps/rtps/attributes/HistoryAttributes.h>
#include <fastrtps/attributes/TopicAttributes.h>
#include <fastrtps/qos/WriterQos.h>
#include <fastrtps/rtps/history/WriterHistory.h>
#include <fastrtps/rtps/attributes/WriterAttributes.h>
#include <fastrtps/rtps/writer/RTPSWriter.h>

#include <ddsrouter_core/types/participant/ParticipantId.hpp>

#include <writer/IWriter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

/**
 * Standard RTPS Writer with less restrictive Attributes.
 */
class Writer : public IWriter
{
private:

    struct RTPSWriterDeleter
    {
        RTPSWriterDeleter()
        {
        }

        void operator ()(
                fastrtps::rtps::RTPSWriter* ptr) const;
    };

public:

    /**
     * @brief Construct a new Writer object
     *
     * Get the Attributes and QoS and create the Writer History and the RTPS Writer.
     *
     * @param participant_id    Participant Id of the Participant that has created this Writer.
     * @param topic             Topic that this Writer subscribes to.
     * @param payload_pool      Shared Payload Pool to received data and take it.
     * @param rtps_participant  RTPS Participant pointer (this is not stored).
     *
     * @throw \c InitializationException in case initialization fails
     */
    Writer(
            const types::ParticipantId& participant_id,
            const types::RealTopic& topic,
            std::shared_ptr<fastrtps::rtps::IPayloadPool>& payload_pool,
            fastrtps::rtps::RTPSParticipant* rtps_participant);

    /**
     * @brief Destroy the Writer object
     *
     * Remove Writer RTPS
     * Remove History
     *
     * @todo Remove every change and release it in IPayloadPool
     */
    virtual ~Writer();

    /////
    // DDS ROUTER METHODS

    /**
     * @brief Write cache change in RTPSWriter's history
     *
     * @param reader_cache_change Cache change coming from a reader
     */
    void write(
            fastrtps::rtps::CacheChange_t* reader_cache_change) noexcept override;

private:

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

    //! RTPS Writer History associated to \c rtps_reader_
    std::unique_ptr<fastrtps::rtps::WriterHistory> rtps_history_;

    //! RTPS Writer
    std::unique_ptr<fastrtps::rtps::RTPSWriter, RTPSWriterDeleter> rtps_writer_;
};

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_RTPS_WRITER_HPP_ */
