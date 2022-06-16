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

#include <reader/IReader.hpp>

#include <ddsrouter_core/types/participant/ParticipantId.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

/**
 * Standard RTPS Reader with less restrictive Attributes.
 *
 * It implements the ReaderListener for itself with \c onNewCacheChangeAdded and \c onReaderMatched callbacks.
 * It implements \c take_and_forward which is the main function run by worker threads
 */
class Reader : public IReader, public fastrtps::rtps::ReaderListener
{
private:

    struct RTPSReaderDeleter
    {
        RTPSReaderDeleter()
        {
        }

        void operator ()(
                fastrtps::rtps::RTPSReader* ptr) const;
    };

public:

    /**
     * @brief Construct a new Reader object
     *
     * Get the Attributes and QoS and create the Reader History and the RTPS Reader.
     *
     * @param participant_id    Router Id of the Participant that has created this Reader.
     * @param topic             Topic that this Reader subscribes to.
     * @param payload_pool      Shared Payload Pool to received data and take it.
     * @param data_forward_queue storing data forward tasks
     * @param rtps_participant  RTPS Participant pointer (this is not stored).
     *
     * @throw \c InitializationException in case any creation has failed
     */
    Reader(
            const types::ParticipantId& participant_id,
            const types::RealTopic& topic,
            std::shared_ptr<fastrtps::rtps::IPayloadPool>& payload_pool,
            DataForwardQueue& data_forward_queue,
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
     * Call the on_data_available_ callback (method \c on_data_available_ from \c IReader ).
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

    /**
     * @brief Take data and forward to writers
     *
     * Take cache changes from the reader history and forward them to all the writers associated to this reader.
     * For each cache change forwarded to writers, forwarded_ variable will be increased by one, so callers of this
     * function will try to match forwarded_ to notified_.
     * This function is accessed by worker threads, and the critical section is protected by a
     * std::atomic_flag take_lock_.
     * The concurrency behavior is equivalent to try-lock semantics, because in general it does not worth to make
     * worker threads wait for the critical section within a reader to be available when there will be plenty of
     * readers whose data is pending to be forwarded. More in detail, (1) a thread that sees the take_lock_ disabled
     * will enable it and enter into de critical section and (2) a thread that sees the take_lock_ enabled will
     * immediately return without trying again. Note that before testing the take_lock_, all threads will increase
     * the notified_ atomic counter by one, so in case a thread fails to enter into the critical section, it would
     * have notified the existence of a new cache change to be taken from the reader history. That means that the
     * thread currently in the loop within the critical section will make an additional iteration for each thread
     * that failed to acquire the take_lock_.
     * In the rare event in which a thread failed to "acquire" the try_lock_ after the thread that acquired it left
     * the loop, it will be a not-forwarded cache change left in the reader: all threads will left this function with
     * a reader state having values of forwarded_ and notified_ such that [forwarded_ + 1 == notified_]. That is not
     * a problem in routing scenarios in which a reader is in generally receiving a continuous stream of messages,
     * because the next call to take_and_forward() will be aware of variables being [forwarded_ + 2 == notified_],
     * so the previously unattended cache change will be finally forwarded.
     */
    void take_and_forward() noexcept override;

protected:

    /////
    // Internal methods

    void enable_() noexcept override;

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
    // VARIABLES

    //! RTPS Reader History associated to \c rtps_reader_
    std::unique_ptr<fastrtps::rtps::ReaderHistory> rtps_history_;

    //! RTPS Reader pointer
    std::unique_ptr<fastrtps::rtps::RTPSReader, RTPSReaderDeleter> rtps_reader_;

    //! Variable tracking the number of times onCacheChangeAdded (if topic reliable) was called while this reader (topic) was disabled
    std::atomic<long> enqueued_while_disabled_;

    //| Variable tracking the number of times take_and_forward is called. Only read and modified within the critical section protected by take_lock_
    std::atomic<unsigned long> notified_;

    //| Variable tracking the number of cache changes forwarded in this reader
    unsigned long forwarded_;

    //! Boolean to control the access to take_and_forward function. Its acquisition and release synchronizes with forwarded_ value
    std::atomic_flag take_lock_;

};

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_RTPS_READER_HPP_ */
