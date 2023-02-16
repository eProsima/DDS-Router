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

#include <mutex>

#include <fastdds/rtps/rtps_fwd.h>
#include <fastrtps/rtps/attributes/HistoryAttributes.h>
#include <fastrtps/attributes/TopicAttributes.h>
#include <fastrtps/qos/ReaderQos.h>
#include <fastrtps/rtps/history/ReaderHistory.h>
#include <fastrtps/rtps/attributes/ReaderAttributes.h>
#include <fastrtps/rtps/reader/RTPSReader.h>
#include <fastrtps/rtps/reader/ReaderListener.h>
#include <fastrtps/utils/TimedMutex.hpp>

#include <ddspipe_core/types/dds/Guid.hpp>
#include <ddspipe_core/types/topic/dds/DdsTopic.hpp>
#include <ddspipe_core/types/participant/ParticipantId.hpp>
#include <ddspipe_core/types/data/RtpsPayloadData.hpp>

#include <ddspipe_participants/library/library_dll.h>
#include <ddspipe_participants/reader/auxiliar/BaseReader.hpp>

namespace eprosima {
namespace ddspipe {
namespace participants {
namespace rtps {

/**
 * Abstract generic class for a RTPS Reader wrapper.
 *
 * It implements the ReaderListener for itself with \c onNewCacheChangeAdded and \c onReaderMatched callbacks.
 *
 * @warning This object is not RAII and must be initialized before used.
 */
class CommonReader : public BaseReader, public fastrtps::rtps::ReaderListener
{
public:

    /////////////////////////
    // CONSTRUCTOR
    /////////////////////////

    // Protected ctor to make class abstract (only built by their childs).

    /**
     * @brief Destroy the CommonReader object
     *
     * Delete the RTPS CommonReader and CommonReader History in case they are set.
     */
    DDSPIPE_PARTICIPANTS_DllAPI virtual ~CommonReader();

    /**
     * @brief Create the internal RTPS Reader.
     *
     * @attention this method should be called right after constructor to create enable internal entities.
     * This is required as this object is a Listener that could be called before finishing construction.
     * Other alternatives have been studied but none have really fit for this case.
     *
     * @throw InitializationException if RTPS Reader creation fails
     *
     * @warning this method is not thread safe.
     * @pre this method can only be called once.
     */
    DDSPIPE_PARTICIPANTS_DllAPI void init();

    /////////////////////////
    // RTPS LISTENER METHODS
    /////////////////////////

    /**
     * @brief CommonReader Listener callback when a new data is added to History
     *
     * This method is call every time a new CacheChange is received by this CommonReader.
     * Filter this same Participant messages.
     * Call the on_data_available_ callback (method \c on_data_available_ from \c BaseReader ).
     *
     * @param [in] change new change received
     */
    DDSPIPE_PARTICIPANTS_DllAPI void onNewCacheChangeAdded(
            fastrtps::rtps::RTPSReader*,
            const fastrtps::rtps::CacheChange_t* const change) noexcept override;

    /**
     * @brief CommonReader Listener callback when a new Writer is matched or unmatched
     *
     * This method is call every time a new Writer is matched or unmatched from this CommonReader.
     * It only creates a log for matching and unmatching (in case it is not a writer from this same Participant)
     *
     * @param [in] info information about the matched Writer
     */
    DDSPIPE_PARTICIPANTS_DllAPI void onReaderMatched(
            fastrtps::rtps::RTPSReader*,
            fastrtps::rtps::MatchingInfo& info) noexcept override;

    /////////////////////////
    // RPC REQUIRED METHODS
    /////////////////////////
    // TODO remove these methods once the double reference is solved

    //! Get GUID of internal RTPS reader
    DDSPIPE_PARTICIPANTS_DllAPI core::types::Guid guid() const noexcept override;

    //! Get internal RTPS reader mutex
    DDSPIPE_PARTICIPANTS_DllAPI fastrtps::RecursiveTimedMutex& get_rtps_mutex() const noexcept override;

    //! Get number of unread cache changes in internal RTPS reader
    DDSPIPE_PARTICIPANTS_DllAPI uint64_t get_unread_count() const noexcept override;

    DDSPIPE_PARTICIPANTS_DllAPI core::types::DdsTopic topic() const noexcept override;

protected:

    /**
     * @brief Construct a new CommonReader object
     *
     * It receives all the attributes and QoS needed to create the internal entities.
     *
     * @note Only protected so only concrete classes are instantiated.
     *
     * @throw \c InitializationException in case any creation has failed
     */
    CommonReader(
            const core::types::ParticipantId& participant_id,
            const core::types::DdsTopic& topic,
            const std::shared_ptr<core::PayloadPool>& payload_pool,
            fastrtps::rtps::RTPSParticipant* rtps_participant,
            const fastrtps::rtps::HistoryAttributes& history_attributes,
            const fastrtps::rtps::ReaderAttributes& reader_attributes,
            const fastrtps::TopicAttributes& topic_attributes,
            const fastrtps::ReaderQos& reader_qos);

    // Specific enable/disable do not need to be implemented

    /**
     * @brief Auxiliary method to create the internal RTPS Reader and History.
     */
    virtual void internal_entities_creation_(
            const fastrtps::rtps::HistoryAttributes& history_attributes,
            const fastrtps::rtps::ReaderAttributes& reader_attributes,
            const fastrtps::TopicAttributes& topic_attributes,
            const fastrtps::ReaderQos& reader_qos);

    /**
     * @brief Return an allocated object
     *
     * @attention this method allocates memory.
     */
    virtual core::types::RtpsPayloadData* create_data_(
            const fastrtps::rtps::CacheChange_t& received_change) const noexcept;

    /**
     * @brief Auxiliary method used in \c take to fill the received data.
     */
    virtual void fill_received_data_(
            const fastrtps::rtps::CacheChange_t& received_change,
            core::types::RtpsPayloadData& data_to_fill) const noexcept;

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
     * Remove this change from CommonReader History and release.
     *
     * @note guard by mutex \c rtps_mutex_
     *
     * @param data : oldest data to take
     * @return \c RETCODE_OK if data has been correctly taken
     * @return \c RETCODE_NO_DATA if there is no data to send
     * @return \c RETCODE_ERROR if there has been an error reading the data or the data read is corrupted
     */
    virtual utils::ReturnCode take_nts_(
            std::unique_ptr<core::IRoutingData>& data) noexcept override;

    /////
    // RTPS specific methods

    /**
     * @brief Default History Attributes to create CommonReader History
     *
     * @return Default HistoryAttributes
     */
    static fastrtps::rtps::HistoryAttributes reckon_history_attributes_(
            const core::types::DdsTopic& topic) noexcept;

    /**
     * @brief Reader Attributes to create RTPS Reader
     */
    static fastrtps::rtps::ReaderAttributes reckon_reader_attributes_(
            const core::types::DdsTopic& topic) noexcept;

    //! Topic Attributes to create RTPS Reader
    static fastrtps::TopicAttributes reckon_topic_attributes_(
            const core::types::DdsTopic& topic) noexcept;

    //! Reader QoS to create RTPS Reader
    static fastrtps::ReaderQos reckon_reader_qos_(
            const core::types::DdsTopic& topic) noexcept;

    /////
    // CommonReader specific methods

    //! Whether a change received is from this Participant (to avoid auto-feedback)
    bool come_from_this_participant_(
            const fastrtps::rtps::CacheChange_t* change) const noexcept;

    //! Whether a guid references this Participant (to avoid auto-feedback)
    bool come_from_this_participant_(
            const fastrtps::rtps::GUID_t guid) const noexcept;

    utils::ReturnCode is_data_correct_(
            const fastrtps::rtps::CacheChange_t* received_change) const noexcept;

    /////
    // EXTERNAL VARIABLES

    //! RTPS Participant
    fastrtps::rtps::RTPSParticipant* rtps_participant_;

    /////
    // INTERNAL VARIABLES

    //!
    core::types::DdsTopic topic_;

    //! RTPS Reader pointer
    fastrtps::rtps::RTPSReader* rtps_reader_;

    //! RTPS Reader History associated to \c rtps_reader_
    fastrtps::rtps::ReaderHistory* rtps_history_;

    //! History attributes to create the History for the internal RTPS Reader.
    fastrtps::rtps::HistoryAttributes history_attributes_;

    //! Reader attributes to create the internal RTPS Reader.
    fastrtps::rtps::ReaderAttributes reader_attributes_;

    //! Topic attributes to create the internal RTPS Reader.
    fastrtps::TopicAttributes topic_attributes_;

    //! Reader QoS to create the internal RTPS Reader.
    fastrtps::ReaderQos reader_qos_;

};

} /* namespace rtps */
} /* namespace participants */
} /* namespace ddspipe */
} /* namespace eprosima */
