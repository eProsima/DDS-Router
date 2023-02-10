// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file DdsBridge.cpp
 *
 */

#include <communication/DdsBridge.hpp>

#include <cpp_utils/exception/UnsupportedException.hpp>
#include <cpp_utils/Log.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {

using namespace eprosima::ddsrouter::core::types;

DdsBridge::DdsBridge(
        const DistributedTopic& topic,
        std::shared_ptr<ParticipantsDatabase> participants_database,
        std::shared_ptr<PayloadPool> payload_pool,
        std::shared_ptr<utils::SlotThreadPool> thread_pool,
        bool enable /* = false */)
    : Bridge(participants_database, payload_pool, thread_pool)
    , topic_(topic)
{
    logDebug(DDSROUTER_DDSBRIDGE, "Creating DdsBridge " << *this << ".");

    std::set<ParticipantId> ids = participants_->get_participants_ids();

    // Generate readers and writers for each participant
    for (ParticipantId id: ids)
    {
        std::shared_ptr<IParticipant> participant = participants_->get_participant(id);

        writers_[id] = participant->create_writer(topic);
        readers_[id] = participant->create_reader(topic);

    }

    // Generate tracks
    for (ParticipantId id: ids)
    {
        // List of all Participants
        std::map<ParticipantId, std::shared_ptr<IWriter>> writers_except_one =
                writers_; // Create a copy of the map

        if (!participants_->get_participant(id)->is_repeater())
        {
            // Remove this Track source participant because it is not repeater
            writers_except_one.erase(id);

            logDebug(
                DDSROUTER_DDSBRIDGE,
                "Not adding own Writer to Track in " << *this << " in Participant " << id << ".");
        }

        // This insert is required as there is no copy method for Track
        // Tracks are always created disabled and then enabled with Bridge enable() method
        tracks_[id] =
                std::make_unique<Track>(
            topic_,
            id,
            readers_[id], std::move(writers_except_one),
            payload_pool_,
            thread_pool,
            false);
    }

    if (enable)
    {
        this->enable();
    }

    logDebug(DDSROUTER_DDSBRIDGE, "DdsBridge " << *this << " created.");
}

DdsBridge::~DdsBridge()
{
    logDebug(DDSROUTER_DDSBRIDGE, "Destroying DdsBridge " << *this << ".");

    // Disable every Track before destruction
    disable();

    // Force deleting tracks before deleting Bridge
    tracks_.clear();

    // Remove all Writers and Readers that were created in construction
    for (ParticipantId id: participants_->get_participants_ids())
    {
        std::shared_ptr<IParticipant> participant = participants_->get_participant(id);
        auto writer = writers_.find(id);
        auto reader = readers_.find(id);

        // Writer and Reader must exist in this Bridge Map for each participant
        assert(writer != writers_.end());
        assert(reader != readers_.end());

        participant->delete_writer(writer->second);
        participant->delete_reader(reader->second);

        writers_.erase(writer);
        readers_.erase(reader);
    }

    // Participants must not be removed as they belong to the Participant Database

    logDebug(DDSROUTER_DDSBRIDGE, "DdsBridge " << *this << " destroyed.");
}

void DdsBridge::enable() noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (!enabled_)
    {
        logInfo(DDSROUTER_DDSBRIDGE, "Enabling DdsBridge for topic " << topic_ << ".");

        // ATTENTION: reference needed or it would copy Track
        for (auto& track_it : tracks_)
        {
            track_it.second->enable();
        }

        enabled_ = true;
    }
}

void DdsBridge::disable() noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (enabled_)
    {
        logInfo(DDSROUTER_DDSBRIDGE, "Disabling DdsBridge for topic " << topic_ << ".");

        // ATTENTION: reference needed or it would copy Track
        for (auto& track_it : tracks_)
        {
            track_it.second->disable();
        }

        enabled_ = false;
    }
}

std::ostream& operator <<(
        std::ostream& os,
        const DdsBridge& bridge)
{
    os << "DdsBridge{" << bridge.topic_ << "}";
    return os;
}

} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
