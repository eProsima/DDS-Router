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
 * @file CommonWriter.hpp
 */

#ifndef __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_RTPS_COMMONWRITER_HPP_
#define __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_RTPS_COMMONWRITER_HPP_

#include <fastdds/rtps/rtps_fwd.h>
#include <fastrtps/rtps/attributes/HistoryAttributes.h>
#include <fastrtps/attributes/TopicAttributes.h>
#include <fastrtps/qos/WriterQos.h>
#include <fastrtps/rtps/history/WriterHistory.h>
#include <fastrtps/rtps/attributes/WriterAttributes.h>
#include <fastrtps/rtps/writer/RTPSWriter.h>
#include <fastrtps/rtps/writer/WriterListener.h>

#include <ddsrouter_core/types/participant/ParticipantId.hpp>
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

using WriteParams = eprosima::fastrtps::rtps::WriteParams;
using SequenceNumber = eprosima::fastrtps::rtps::SequenceNumber_t;

/**
 * Abstract generic class for a RTPS Writer wrapper.
 *
 * It implements the WriterListener for itself with \c onWriterMatched callbacks.
 *
 * @todo remove those variables that are not needed (e.g. rtps_participant_)
 */
class CommonWriter : public BaseWriter, public fastrtps::rtps::WriterListener
{
public:

    /**
     * @brief Destroy the Writer object
     *
     * Delete the RTPS Writer and Writer History in case they are set.
     */
    virtual ~CommonWriter();

protected:

    /**
     * @brief Construct a new Writer object
     *
     * It receives all the attributes and QoS needed to create the internal entities.
     *
     * @note Only protected so only concrete classes are instantiated.
     *
     * @throw \c InitializationException in case any creation has failed
     */
    CommonWriter(
            const types::ParticipantId& participant_id,
            const types::DdsTopic& topic,
            std::shared_ptr<PayloadPool> payload_pool,
            fastrtps::rtps::RTPSParticipant* rtps_participant,
            const bool repeater,
            const fastrtps::rtps::HistoryAttributes& history_attributes,
            const fastrtps::rtps::WriterAttributes& writer_attributes,
            const fastrtps::TopicAttributes& topic_attributes,
            const fastrtps::WriterQos& writer_qos,
            const utils::PoolConfiguration& pool_configuration);

    // Specific enable/disable do not need to be implemented

    /**
     * @brief Write specific method
     *
     * Store new data as message to send (synchronously) (it could use PayloadPool to not copy payload).
     * Take next Untaken Change.
     * Set \c data with the message taken (data payload must be stored from PayloadPool).
     * Remove this change from Reader History and release.
     *
     * It does not require mutex, it will be guarded by RTPS CommonWriter mutex in internal methods.
     *
     * @param data : oldest data to take
     * @return \c RETCODE_OK if data has been correctly taken
     * @return \c RETCODE_NO_DATA if \c data is empty
     * @return \c RETCODE_ERROR if error occurred
     */
    virtual utils::ReturnCode write_(
            std::unique_ptr<types::DataReceived>& data) noexcept override;

    /**
     * @brief Auxiliary method used in \c write to fill the cache change to send.
     *
     * @param [out] to_send_change_to_fill cache change to be filled and sent.
     * @param [out] to_send_params write params to be filled and sent.
     * @param [in] data data received that must be sent.
     */
    virtual utils::ReturnCode fill_to_send_data_(
            fastrtps::rtps::CacheChange_t* to_send_change_to_fill,
            eprosima::fastrtps::rtps::WriteParams& to_send_params,
            std::unique_ptr<types::DataReceived>& data) const noexcept;

    /**
     * @brief Auxiliary method used after \c write to fill data value.
     *
     * @param [in] to_send_params write params of the cache change sent.
     * @param [out] data data to be fulfilled with params.
     */
    virtual void fill_sent_data_(
            const eprosima::fastrtps::rtps::WriteParams& params,
            std::unique_ptr<types::DataReceived>& data_to_fill) const noexcept;

    /////
    // RTPS specific methods

    /**
     * @brief Create \c this->rtps_writer_ internal object
     *
     * @note this method exists because it may be created from child classes, thus the methods may not be yet available
     * to get atts and qos.
     *
     * @param rtps_participant
     */
    void internal_entities_creation_(
            const fastrtps::rtps::HistoryAttributes& history_attributes,
            const fastrtps::rtps::WriterAttributes& writer_attributes,
            const fastrtps::TopicAttributes& topic_attributes,
            const fastrtps::WriterQos& writer_qos,
            const utils::PoolConfiguration& pool_configuration);

    /**
     * @brief History Attributes to create RTPS Writer History
     */
    static fastrtps::rtps::HistoryAttributes history_attributes_(
            const types::DdsTopic& topic) noexcept;

    /**
     * @brief Writer Attributes to create RTPS Writer
     */
    static fastrtps::rtps::WriterAttributes writer_attributes_(
            const types::DdsTopic& topic) noexcept;

    //! Topic Attributes to create RTPS Writer
    static fastrtps::TopicAttributes topic_attributes_(
            const types::DdsTopic& topic) noexcept;

    //! QoS for RTPS Writer
    static fastrtps::WriterQos writer_qos_(
            const types::DdsTopic& topic) noexcept;

    //! Cache Change Pool Configuration
    static utils::PoolConfiguration cache_change_pool_configuration_(
            const types::DdsTopic& topic) noexcept;

    //! Whether a guid references this Participant
    bool come_from_this_participant_(
            const fastrtps::rtps::GUID_t guid) const noexcept;

    /////
    // EXTERNAL VARIABLES

    //! RTPS Participant
    fastrtps::rtps::RTPSParticipant* rtps_participant_;

    //! Wether it is repeater or not (used for data filters and/or qos)
    bool repeater_;

    /////
    // INTERNAL VARIABLES

    //! RTPS CommonWriter pointer
    fastrtps::rtps::RTPSWriter* rtps_writer_;

    //! RTPS CommonWriter History associated to \c rtps_reader_
    fastrtps::rtps::WriterHistory* rtps_history_;

    //! Data Filter used to filter cache changes at the RTPSWriter level.
    std::unique_ptr<fastdds::rtps::IReaderDataFilter> data_filter_;
};

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_RTPS_COMMONWRITER_HPP_ */
