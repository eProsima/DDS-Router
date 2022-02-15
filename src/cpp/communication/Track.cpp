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

#include <ddsrouter/communication/Track.hpp>
#include <ddsrouter/exceptions/UnsupportedException.hpp>
#include <ddsrouter/types/Log.hpp>

namespace eprosima {
namespace ddsrouter {

Track::Track(
        const RealTopic& topic,
        ParticipantId reader_participant_id,
        std::shared_ptr<IReader> reader,
        std::map<ParticipantId, std::shared_ptr<IWriter>>&& writers,
        std::shared_ptr<PayloadPool> payload_pool,
        bool enable /* = false */) noexcept
    : reader_participant_id_(reader_participant_id)
    , topic_(topic)
    , reader_(reader)
    , writers_(writers)
    , payload_pool_(payload_pool)
    , enabled_(false)
    , exit_(false)
    , is_data_available_(false)
{
    logDebug(DDSROUTER_TRACK, "Creating Track " << *this << ".");

    // Set this track to on_data_available lambda call
    reader_->set_on_data_available_callback(std::bind(&Track::data_available, this));

    // Activate transmit thread even without being enabled
    transmit_thread_ = std::thread(&Track::transmit_thread_function_, this);

    if (enable)
    {
        // Activate Track
        this->enable();
    }
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
    {
        // Set exit status and call transmit thread to awake and terminate. Then wait for it.
        std::lock_guard<std::mutex> lock(data_available_mutex_);
        exit_.store(true); // This is not needed to be guarded as it is atomic
    }

    data_available_condition_variable_.notify_all();
    transmit_thread_.join();

    logDebug(DDSROUTER_TRACK, "Track " << *this << " destroyed.");
}

void Track::enable() noexcept
{
    std::lock_guard<std::recursive_mutex> lock(track_mutex_);

    if (!enabled_)
    {
        logInfo(DDSROUTER_TRACK, "Enabling Track " << reader_participant_id_ << " for topic " << topic_ << ".");

        // Enabling writers
        for (auto& writer_it : writers_)
        {
            writer_it.second->enable();
        }

        // Enabling reader
        reader_->enable();

        enabled_ = true;
    }
}

void Track::disable() noexcept
{
    std::lock_guard<std::recursive_mutex> lock(track_mutex_);

    if (enabled_)
    {
        logInfo(DDSROUTER_TRACK, "Disabling Track " << reader_participant_id_ << " for topic " << topic_ << ".");

        // Do disable before stop in the mutex so the Track is forced to stop in next iteration
        enabled_ = false;
        {
            // Stop if there is a transmission in course till the data is sent
            std::unique_lock<std::mutex> lock(on_transmition_mutex_);
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

void Track::no_more_data_available_() noexcept
{
    std::lock_guard<std::mutex> lock(data_available_mutex_);
    logDebug(DDSROUTER_TRACK, "Track " << *this << " has no more data to send.");
    is_data_available_.store(false);
}

bool Track::should_transmit_nts_() noexcept
{
    return !exit_ && enabled_ && this->is_data_available_;
}

void Track::data_available() noexcept
{
    // Only hear callback if it is enabled
    if (enabled_)
    {
        logDebug(DDSROUTER_TRACK, "Track " << *this << " has data ready to be sent.");

        // It does need to guard the mutex to avoid notifying Track thread while it is checking variable condition
        {
            // Set data available to true and notify transmit thread
            std::lock_guard<std::mutex> lock(data_available_mutex_);
            is_data_available_.store(true);
        }

        data_available_condition_variable_.notify_one();
    }
}

void Track::transmit_thread_function_() noexcept
{
    while (!exit_)
    {
        // Wait in Condition Variable till there is data to send or it must exit
        {
            std::unique_lock<std::mutex> lock(data_available_mutex_);
            data_available_condition_variable_.wait(
                lock,
                [this]
                {
                    return this->is_data_available_ || this->exit_;
                });
        }

        // Once thread awakes, transmit without any mutex guarded
        transmit_();
    }
}

void Track::transmit_() noexcept
{
    // Loop that ends if it should stop transmitting (should_transmit_nts_).
    // Called inside the loop so it is protected by a mutex that is freed in every iteration.
    while (true)
    {
        // Lock Mutex on_transmition while a data is being transmitted
        std::unique_lock<std::mutex> lock(on_transmition_mutex_);

        // If it must not keep transmitting, stop loop
        if (!should_transmit_nts_())
        {
            break;
        }

        // Get data received
        std::unique_ptr<DataReceived> data = std::make_unique<DataReceived>();
        ReturnCode ret = reader_->take(data);

        if (ret == ReturnCode::RETCODE_NO_DATA)
        {
            // There is no more data, so finish loop and wait again for new data
            no_more_data_available_();
            break;
        }
        else if (!ret)
        {
            // Error reading data
            logWarning(DDSROUTER_TRACK, "Error taking data in Track " << topic_ << ". Error code " << ret
                                                                      << ". Skipping data and continue.");
            continue;
        }

        logDebug(DDSROUTER_TRACK,
                "Track " << reader_participant_id_ << " for topic " << topic_ <<
                " transmitting data from remote endpoint " << data->source_guid << ".");

        // Send data through writers
        for (auto& writer_it : writers_)
        {
            ret = writer_it.second->write(data);

            if (!ret)
            {
                logWarning(DDSROUTER_TRACK, "Error writting data in Track " << topic_ << ". Error code "
                                                                            << ret <<
                        ". Skipping data for this writer and continue.");
                continue;
            }
        }

        payload_pool_->release_payload(data->payload);
    }
}

std::ostream& operator <<(
        std::ostream& os,
        const Track& track)
{
    os << "Track{" << track.topic_ << ";" << track.reader_participant_id_ << "}";
    return os;
}

} /* namespace ddsrouter */
} /* namespace eprosima */
