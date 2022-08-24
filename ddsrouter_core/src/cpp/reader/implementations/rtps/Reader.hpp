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
 * @file Reader.hpp
 */

#ifndef __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_RTPS_READER_HPP_
#define __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_RTPS_READER_HPP_

#include <fastdds/rtps/rtps_fwd.h>
#include <fastrtps/rtps/attributes/HistoryAttributes.h>
#include <fastrtps/attributes/TopicAttributes.h>
#include <fastrtps/qos/ReaderQos.h>
#include <fastrtps/rtps/history/ReaderHistory.h>
#include <fastrtps/rtps/attributes/ReaderAttributes.h>
#include <fastrtps/rtps/reader/RTPSReader.h>
#include <fastrtps/rtps/reader/ReaderListener.h>

#include <ddsrouter_core/types/participant/ParticipantId.hpp>

#include <reader/implementations/auxiliar/BaseReader.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

/**
 * Standard RTPS Reader with less restrictive Attributes.
 *
 * It implements the ReaderListener for itself with \c onNewCacheChangeAdded and \c onReaderMatched callbacks.
 */
class Reader : public BaseReader, public fastrtps::rtps::ReaderListener
{
public:

    /**
     * @brief Construct a new Reader object
     *
     * Get the Attributes and QoS and create the Reader History and the RTPS Reader.
     *
     * @param participant_id    Router Id of the Participant that has created this Reader.
     * @param topic             Topic that this Reader subscribes to.
     * @param payload_pool      Shared Payload Pool to received data and take it.
     * @param rtps_participant  RTPS Participant pointer (this is not stored).
     *
     * @throw \c InitializationException in case any creation has failed
     */
    Reader(
            const types::ParticipantId& participant_id,
            const types::RealTopic& topic,
            std::shared_ptr<PayloadPool> payload_pool,
            fastrtps::rtps::RTPSParticipant* rtps_participant);

    /**
     * @brief Destroy the Reader object
     *
     * Delete the RTPS Reader and Reader History in case they are set.
     */
    virtual ~Reader();

    /////
    // LISTENER METHODS

    /**
     * @brief Reader Listener callback when a new data is added to History
     *
     * This method is call every time a new CacheChange is received by this Reader.
     * Filter this same Participant messages.
     * Call the on_data_available_ callback (method \c on_data_available_ from \c BaseReader ).
     *
     * @param [in] change new change received
     */
    void onNewCacheChangeAdded(
            fastrtps::rtps::RTPSReader*,
            const fastrtps::rtps::CacheChange_t* const change) noexcept override;

    /**
     * @brief Reader Listener callback when a new Writer is matched or unmatched
     *
     * This method is call every time a new Writer is matched or unmatched from this Reader.
     * It only creates a log for matching and unmatching (in case it is not a writer from this same Participant)
     *
     * @param [in] info information about the matched Writer
     */
    void onReaderMatched(
            fastrtps::rtps::RTPSReader*,
            fastrtps::rtps::MatchingInfo& info) noexcept override;

protected:

    // Specific enable/disable do not need to be implemented

    /**
     * @brief Enable specific method for RTPS reader
     *
     * Check if there is data available to read
     */

    void enable_() noexcept;

    /**
     * @brief Take specific method
     *
     * Check if there are messages to take.
     * Take next Untaken Change.
     * Set \c data with the message taken (data payload must be stored from PayloadPool).
     * Remove this change from Reader History and release.
     *
     * @note guard by mutex \c rtps_mutex_
     *
     * @param data : oldest data to take
     * @return \c RETCODE_OK if data has been correctly taken
     * @return \c RETCODE_NO_DATA if \c data_to_send_ is empty
     * @return \c RETCODE_NO_DATA if \c data_to_send_ is empty
     */
    utils::ReturnCode take_(
            std::unique_ptr<types::DataReceived>& data) noexcept override;

    /////
    // RTPS specific methods

    /**
     * @brief Default History Attributes to create Reader History
     *
     * @return Default HistoryAttributes
     */
    fastrtps::rtps::HistoryAttributes history_attributes_() const noexcept;

    /**
     * @brief Default Reader Attributes to create Reader
     *
     * It returns the less restrictive Attributes for a Reader
     * durability: VOLATILE
     * reliability: BEST_EFFORT
     *
     * @return Default ReaderAttributes
     */
    fastrtps::rtps::ReaderAttributes reader_attributes_() const noexcept;

    //! Default Topic Attributes to create Reader
    fastrtps::TopicAttributes topic_attributes_() const noexcept;

    //! Default QoS Reader (must be the same as the attributes)
    fastrtps::ReaderQos reader_qos_() const noexcept;

    /////
    // Reader specific methods

    //! Whether a change received is from this Participant (to avoid auto-feedback)
    bool accept_message_from_this_source_(
            const fastrtps::rtps::CacheChange_t* change) const noexcept;

    //! Whether a guid references this Participant (to avoid auto-feedback)
    bool accept_message_from_this_source_(
            const fastrtps::rtps::GUID_t guid) const noexcept;

    /////
    // VARIABLES

    //! RTPS Reader pointer
    fastrtps::rtps::RTPSReader* rtps_reader_;

    //! RTPS Reader History associated to \c rtps_reader_
    fastrtps::rtps::ReaderHistory* rtps_history_;

    //! Mutex that guards every access to the RTPS Reader
    mutable std::recursive_mutex rtps_mutex_;
};

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_RTPS_READER_HPP_ */
