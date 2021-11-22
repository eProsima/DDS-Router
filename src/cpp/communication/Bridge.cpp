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
 * @file Bridge.cpp
 *
 */

#include <ddsrouter/communication/Bridge.hpp>
#include <ddsrouter/exceptions/UnsupportedException.hpp>
#include <ddsrouter/types/Log.hpp>

namespace eprosima {
namespace ddsrouter {

// TODO: Add logs

Bridge::Bridge(
        const RealTopic& topic,
        std::shared_ptr<ParticipantDatabase> participants_database,
        bool enable /* = false */)
    : topic_(topic)
    , participants_(participants_database)
    , enabled_(false)
{
    std::vector<ParticipantId> ids = participants_->get_participant_ids();

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

        // Get this Track source participant before removing it from map
        writers_except_one.erase(id); // TODO: check if this element is removed in erase or if source is still valid

        // This insert is required as there is no copy method for Track
        // Tracks are always created disabled and then enabled with Bridge enable() method
        tracks_[id] =
                std::make_unique<Track>(topic_, id, readers_[id], std::move(writers_except_one), false);
    }

    if (enable)
    {
        this->enable();
    }
}

Bridge::~Bridge()
{
    // Disable every Track before destruction
    disable();

    // Force deleting tracks before deleting Bridge
    tracks_.clear();

    // Remove all Writers and Readers that were created in construction
    for (ParticipantId id: participants_->get_participant_ids())
    {
        std::shared_ptr<IParticipant> participant = participants_->get_participant(id);
        auto writer = writers_.find(id);
        auto reader = readers_.find(id);

        // Writer and Reader must exist in this Bridge Map for each participant
        assert(writer != writers_.end());
        assert(reader != readers_.end());

        participant->delete_writer(writer->second);
        participant->delete_reader(reader->second);
    }

    // Participants must not be removed as they belong to the Participant Database
}

void Bridge::enable() noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (!enabled_)
    {
        logInfo(DDSROUTER_BRIDGE, "Enabling Bridge for topic " << topic_ << ".");

        // ATTENTION: reference needed or it would copy Track
        for (auto& track_it : tracks_)
        {
            track_it.second->enable();
        }

        enabled_ = true;
    }
}

void Bridge::disable() noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (enabled_)
    {
        logInfo(DDSROUTER_BRIDGE, "Disabling Bridge for topic " << topic_ << ".");

        // ATTENTION: reference needed or it would copy Track
        for (auto& track_it : tracks_)
        {
            track_it.second->disable();
        }

        enabled_ = false;
    }
}

} /* namespace ddsrouter */
} /* namespace eprosima */
