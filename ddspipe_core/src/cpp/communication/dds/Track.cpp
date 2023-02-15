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
 * @file Track.cpp
 *
 */

#include <cpp_utils/exception/UnsupportedException.hpp>
#include <cpp_utils/Log.hpp>
#include <cpp_utils/thread_pool/pool/SlotThreadPool.hpp>
#include <cpp_utils/thread_pool/task/TaskId.hpp>

#include <ddspipe_core/communication/dds/Track.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {

using namespace eprosima::ddspipe::core::types;

const unsigned int Track::MAX_MESSAGES_TRANSMIT_LOOP_ = 100;

Track::Track(
        const utils::Heritable<types::DistributedTopic>& topic,
        const types::ParticipantId& reader_participant_id,
        const std::shared_ptr<IReader>& reader,
        std::map<types::ParticipantId, std::shared_ptr<IWriter>>&& writers,
        const std::shared_ptr<PayloadPool>& payload_pool,
        const std::shared_ptr<utils::SlotThreadPool>& thread_pool) noexcept
    : topic_(topic)
    , reader_participant_id_(reader_participant_id)
    , reader_(reader)
    , writers_(std::move(writers))
    , payload_pool_(payload_pool)
    , enabled_(false)
    , exit_(false)
    , data_available_status_(DataAvailableStatus::no_more_data)
    , transmit_task_id_(utils::new_unique_task_id())
    , thread_pool_(thread_pool)
{
    logDebug(DDSROUTER_TRACK, "Creating Track " << *this << ".");

    // Set this track to on_data_available lambda call
    reader_->set_on_data_available_callback(std::bind(&Track::data_available_, this));

    // Set slot in thread pool
    thread_pool_->slot(
        transmit_task_id_,
        std::bind(&Track::transmit_, this));

    logDebug(DDSROUTER_TRACK, "Track " << *this << " created.");
}

Track::~Track()
{
    logDebug(DDSROUTER_TRACK, "Destroying Track " << *this << ".");

    // Disable reader and writers
    disable();

    // Unset callback on the Reader (this is needed as Reader will live longer than Track)
    reader_->unset_on_data_available_callback();

    // It does need to guard the mutex to avoid notifying Track thread while it is checking variable condition
    // Set exit status and call transmit thread to awake and terminate. Then wait for it.
    exit_.store(true);

    logDebug(DDSROUTER_TRACK, "Track " << *this << " destroyed.");
}

void Track::enable() noexcept
{
    std::lock_guard<std::mutex> lock(track_mutex_);

    if (!enabled_)
    {
        logInfo(DDSROUTER_TRACK, "Enabling Track " << reader_participant_id_ << " for topic " << topic_->topic_name() << ".");
        enabled_ = true;

        // Enable writers before reader, to avoid starting a transmission (not protected with \c track_mutex_) which may
        // attempt to write with a yet disabled writer

        // Enabling writers
        for (auto& writer_it : writers_)
        {
            writer_it.second->enable();
        }

        // Enabling reader
        reader_->enable();

    }
}

void Track::disable() noexcept
{
    std::lock_guard<std::mutex> lock(track_mutex_);

    if (enabled_)
    {
        logInfo(DDSROUTER_TRACK, "Disabling Track " << reader_participant_id_ << " for topic " << topic_->topic_name() << ".");

        // Do disable before stop in the mutex so the Track is forced to stop in next iteration
        enabled_ = false;
        {
            // Stop if there is a transmission in course till the data is sent
            std::unique_lock<std::mutex> lock(on_transmission_mutex_);
        }

        // Disabling Reader
        reader_->disable();

        // Disabling Writers
        for (auto& writer_it : writers_)
        {
            writer_it.second->disable();
        }
    }
}

bool Track::should_transmit_() noexcept
{
    return !exit_ && enabled_;
}

void Track::data_available_() noexcept
{
    // Only hear callback if it is enabled
    if (enabled_)
    {
        logDebug(DDSROUTER_TRACK, "Track " << *this << " has data ready to be sent.");

        // Get previous status and set current one to >=2 (it it was already >=2 it will keep being >2)
        unsigned int previous_status = data_available_status_.fetch_add(DataAvailableStatus::new_data_arrived);

        if (previous_status == DataAvailableStatus::no_more_data)
        {
            // no_more_data was set as current status, so no thread was running
            // (and will not start as 2 is set as new current status)
            thread_pool_->emit(transmit_task_id_);
            logDebug(DDSROUTER_TRACK, "Track " << *this << " send callback to queue.");
        }
    }
}

void Track::transmit_() noexcept
{
    // Loop that ends if it should stop transmitting (should_transmit_nts_).
    // Called inside the loop so it is protected by a mutex that is freed in every iteration.

    // Lock Mutex on_transmition while a data is being transmitted
    // This prevents the Track to be disabled (and disable writers and readers) while sending a data
    // enabled_ will be set to false before taking the mutex, so the track will finish after current iteration
    std::unique_lock<std::mutex> lock(on_transmission_mutex_);

    // TODO: Count the times it loops to break it at some point if needed
    while (should_transmit_())
    {
        // It starts transmitting, so it sets the data available status as transmitting
        // This will erase every previous value added in on_data_available and set 1
        data_available_status_.store(DataAvailableStatus::transmitting_data);

        // Get data received (send empty data to be created(allocated) in reader)
        std::unique_ptr<IRoutingData> data;
        utils::ReturnCode ret = reader_->take(data);

        if (ret == utils::ReturnCode::RETCODE_NO_DATA)
        {
            // There is no more data, so reduce in 1 the status
            unsigned int previous_status = data_available_status_.fetch_sub(DataAvailableStatus::transmitting_data);
            if (previous_status == DataAvailableStatus::transmitting_data)
            {
                // Previous Status = 1 (transmitting => no on_data_available callback has been called)
                // Current Status  = 0 (new callbacks will emit the task)
                // => close this thread and keeps status as 0 so new data available emit task
                break;
            }
            else
            {
                // New data has arrived while setting no_more_data, so it should continue
                // While setting status to 1 again, the value is still >=1 so no other thread will start
                continue;
            }
        }
        else if (!ret)
        {
            // Error reading data
            logWarning(DDSROUTER_TRACK, "Error taking data in Track " << topic_->topic_name() << ". Error code " << ret
                                                                      << ". Skipping data and continue.");
            continue;
        }

        logDebug(DDSROUTER_TRACK,
                "Track " << reader_participant_id_ << " for topic " << topic_->topic_name() <<
                " transmitting data from remote endpoint.");

        // Send data through writers
        for (auto& writer_it : writers_)
        {
            logDebug(
                DDSROUTER_TRACK,
                "Forwarding data to writer " << writer_it.first << ".");

            ret = writer_it.second->write(*data);

            if (!ret)
            {
                logWarning(DDSROUTER_TRACK, "Error writting data in Track " << topic_->topic_name() << ". Error code "
                                                                            << ret <<
                        ". Skipping data for this writer and continue.");
                continue;
            }
        }

        // Let the data to be removed by itself
    }
}

std::ostream& operator <<(
        std::ostream& os,
        const Track& track)
{
    os << "Track{" << track.topic_->topic_name() << ";" << track.reader_participant_id_ << "}";
    return os;
}

} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
