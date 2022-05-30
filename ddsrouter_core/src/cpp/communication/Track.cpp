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

#include <ddsrouter_utils/exception/UnsupportedException.hpp>
#include <ddsrouter_utils/Log.hpp>

#include <communication/Track.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

using namespace eprosima::ddsrouter::core::types;

Track::Track(
        const RealTopic& topic,
        ParticipantId reader_participant_id,
        std::shared_ptr<IReader> reader,
        std::map<ParticipantId, std::shared_ptr<IWriter>>&& writers,
        std::shared_ptr<PayloadPool> payload_pool,
        std::shared_ptr<thread::ThreadPoolManager> thread_pool,
        bool enable /* = false */) noexcept
    : reader_participant_id_(reader_participant_id)
    , topic_(topic)
    , reader_(reader)
    , writers_(writers)
    , payload_pool_(payload_pool)
    , enabled_(false)
    , exit_(false)
    , thread_pool_(thread_pool)
{
    logDebug(DDSROUTER_TRACK, "Creating Track " << *this << ".");

    // Set this track to on_data_available lambda call
    reader_->set_on_data_available_callback(std::bind(&Track::data_available_, this));

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

    logDebug(DDSROUTER_TRACK, "Track " << *this << " destroyed.");
}

void Track::enable() noexcept
{
    std::lock_guard<std::recursive_mutex> lock(track_mutex_);

    if (!enabled_)
    {
        logInfo(DDSROUTER_TRACK, "Enabling Track " << reader_participant_id_ << " for topic " << topic_ << ".");
        enabled_ = true;

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
    std::lock_guard<std::recursive_mutex> lock(track_mutex_);

    if (enabled_)
    {
        logInfo(DDSROUTER_TRACK, "Disabling Track " << reader_participant_id_ << " for topic " << topic_ << ".");

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

void Track::data_available_() noexcept
{
    // Only hear callback if it is enabled
    if (enabled_)
    {
        logDebug(DDSROUTER_TRACK, "Track " << *this << " has data ready to be sent.");

        // Emit to call transmit from thread pool
        thread_pool_->emit([this](){ Track::transmit_(); });
    }
}

void Track::transmit_() noexcept
{
    // TODO: this will only work if there is 1 on_data_available callback for message

    // Lock Mutex on_transmition while a data is being transmitted
    // This prevents the Track to be disabled (and disable writers and readers) while sending a data
    // std::unique_lock<std::mutex> lock(on_transmission_mutex_);
    if (on_transmission_mutex_.try_lock())
    {
        uint8_t n_iter = 0;
        while (n_iter < 10)
        {
            // If this Track should exit, do nothing
            if (exit_)
            {
                on_transmission_mutex_.unlock();
                return;
            }

            // Get data received
            std::unique_ptr<DataReceived> data = std::make_unique<DataReceived>();
            utils::ReturnCode ret = reader_->take(data);

            if (ret == utils::ReturnCode::RETCODE_NO_DATA)
            {
                if (!n_iter)
                {
                    // Should not call this method if there is no data to read
                    logWarning(DDSROUTER_TRACK, "Error taking data in Track " << topic_ << " from reader without data.");
                }
                on_transmission_mutex_.unlock();
                return;
            }
            else if (ret == utils::ReturnCode::RETCODE_NOT_ENABLED)
            {
                // Should not call this method if there is no data to read
                logWarning(DDSROUTER_TRACK, "Error taking data in Track " << topic_ << " from not enabled reader.");
                on_transmission_mutex_.unlock();
                return;
            }
            else if (!ret)
            {
                // Error reading data
                logWarning(DDSROUTER_TRACK, "Error taking data in Track " << topic_ << ". Error code " << ret
                                                                            << ". Skipping data and continue.");
                on_transmission_mutex_.unlock();
                return;
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
            n_iter++;
        }
    }
    else
    {
        return;
    }
}

std::ostream& operator <<(
        std::ostream& os,
        const Track& track)
{
    os << "Track{" << track.topic_ << ";" << track.reader_participant_id_ << "}";
    return os;
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
