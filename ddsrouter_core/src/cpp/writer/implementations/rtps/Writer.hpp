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
 * Standard RTPS Writer with less restrictive Attributes.
 */
class
    Writer : public BaseWriter, public fastrtps::rtps::WriterListener
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

    /**
     * @brief Write with parameters, and copy to \c sequenceNumber the sequence number associated to the last write operation
     *
     * This method calls the protected method \c writer_ to make the actual write function.
     *
     * Thread safe with mutex \c mutex_ .
     *
     * @param data : oldest data to take
     * @param wparams : parameters to use in write operation
     * @param sequenceNumber : reference where to copy the sequence number associated to the write operation
     * @return \c RETCODE_OK if data has been correctly taken
     * @return \c RETCODE_NO_DATA if \c data is empty
     * @return \c RETCODE_ERROR if error occurred
     *
     */
    utils::ReturnCode write(
            std::unique_ptr<types::DataReceived>& data,
            WriteParams& wparams,
            SequenceNumber& sequenceNumber) noexcept;

    /**
     * @brief Write with parameters
     *
     * This method calls the overloaded method \c write(std::unique_ptr<types::DataReceived>&, WriteParams&, SequenceNumber&)
     *
     * @param data : oldest data to take
     * @param wparams : parameters to use in write operation
     * @return \c RETCODE_OK if data has been correctly taken
     * @return \c RETCODE_NO_DATA if \c data is empty
     * @return \c RETCODE_ERROR if error occurred
     *
     */
    utils::ReturnCode write(
            std::unique_ptr<types::DataReceived>& data,
            WriteParams& wparams) noexcept;

    //! Override onWriterMatched with debug proposes
    void onWriterMatched(
            fastrtps::rtps::RTPSWriter*,
            fastrtps::rtps::MatchingInfo& info) noexcept override;

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
     * @return \c RETCODE_NO_DATA if \c data is empty
     * @return \c RETCODE_ERROR if error occurred
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
    // fastrtps::rtps::WriterAttributes writer_attributes_() const noexcept;
    // TMP: until Transparency module is available
    fastrtps::rtps::WriterAttributes writer_attributes_() noexcept;

    //! Default Topic Attributes to create Writer
    fastrtps::TopicAttributes topic_attributes_() const noexcept;

    //! Default QoS Writer (must be the same as the attributes)
    // fastrtps::WriterQos writer_qos_() const noexcept;
    // TMP: until Transparency module is available
    fastrtps::WriterQos writer_qos_() noexcept;

    //! Default Cache Change Pool Configuration
    utils::PoolConfiguration cache_change_pool_configuration_() const noexcept;

    //! Whether a guid references this Participant (to avoid auto-feedback)
    bool come_from_this_participant_(
            const fastrtps::rtps::GUID_t guid) const noexcept;

    /////
    // VARIABLES

    //! RTPS Writer pointer
    fastrtps::rtps::RTPSWriter* rtps_writer_;

    //! RTPS Writer History associated to \c rtps_reader_
    fastrtps::rtps::WriterHistory* rtps_history_;

    //! Data Filter used to filter cache changes at the RTPSWriter level.
    std::unique_ptr<fastdds::rtps::IReaderDataFilter> data_filter_;

    //! Whether this writer belongs to a repeater participant
    bool repeater_;

    //! Whether to write with parameters given by attribute \c writer_info_
    bool write_with_params_;

    /**
     * Struct storing the sequence number associated to the last write operation, and the parameters with which to
     * perform a write operation if flag \c write_with_params_ is set
     */
    struct WriteInfo
    {
        WriteParams write_params;
        SequenceNumber sequence_number;
    }
    write_info_;
};

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_RTPS_WRITER_HPP_ */
