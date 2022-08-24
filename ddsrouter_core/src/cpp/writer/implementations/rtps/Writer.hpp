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

#include <ddsrouter_utils/types/Atomicable.hpp>

#include <ddsrouter_core/types/participant/ParticipantId.hpp>
#include <ddsrouter_core/types/common/types.hpp>
#include <writer/implementations/auxiliar/BaseWriter.hpp>
#include <efficiency/cache_change/CacheChangePool.hpp>

/////
// Forward declarations
namespace eprosima {
namespace ddsrouter {
namespace core {

struct CacheChangePoolConfiguration;

} /* namespace core */
} /* namespace ddsrouter */

namespace fastdds {
namespace rtps {

struct IReaderDataFilter;

} /* namespace rtps */
} /* namespace fastdds */
} /* namespace eprosima */

namespace eprosima {
namespace ddsrouter {
namespace core {
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
            const types::ParticipantId& participant_id,
            const types::RealTopic& topic,
            std::shared_ptr<PayloadPool> payload_pool,
            fastrtps::rtps::RTPSParticipant* rtps_participant,
            unsigned int max_history_depth,
            std::shared_ptr<types::GuidPrefixDataFilterType> target_guids_filter,
            const bool repeater = false);

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
    virtual utils::ReturnCode write_(
            std::unique_ptr<types::DataReceived>& data) noexcept override;

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

    //! Default Cache Change Pool Configuration
    utils::PoolConfiguration cache_change_pool_configuration_() const noexcept;

    /////
    // VARIABLES

    //! RTPS Writer pointer
    fastrtps::rtps::RTPSWriter* rtps_writer_;

    //! RTPS Writer History associated to \c rtps_reader_
    fastrtps::rtps::WriterHistory* rtps_history_;

    //! Data Filter used to filter cache changes at the RTPSWriter level.
    std::unique_ptr<fastdds::rtps::IReaderDataFilter> data_filter_;

    bool repeater_;

    //! Collects these guid prefixes that the Writer must filter when sending data.
    std::shared_ptr<types::GuidPrefixDataFilterType> target_guids_filter_;
};

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_RTPS_WRITER_HPP_ */
