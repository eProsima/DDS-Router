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
 * @file ServiceRegistry.cpp
 *
 */

#include <regex>
#include <string>

#include <cpp_utils/Log.hpp>

#include <communication/rpc/ServiceRegistry.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {

using namespace eprosima::ddspipe::core::types;

const unsigned int ServiceRegistry::DEFAULT_MAX_ENTRIES_ = 5000;

ServiceRegistry::ServiceRegistry(
        const RpcTopic& topic,
        const ParticipantId& participant_id)
    : topic_(topic)
    , participant_id_(participant_id)
    , enabled_(false)
{
    logDebug(DDSROUTER_SERVICEREGISTRY,
            "ServiceRegistry created for service " << topic <<
            " in participant " << participant_id << ".");
}

void ServiceRegistry::enable() noexcept
{
    enabled_ = true;
}

void ServiceRegistry::disable() noexcept
{
    enabled_ = false;
}

bool ServiceRegistry::enabled() const noexcept
{
    return enabled_;
}

void ServiceRegistry::add(
        SequenceNumber idx,
        std::pair<ParticipantId, SampleIdentity> new_entry) noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (registry_.count(idx))
    {
        // Should never occur as each sequence number associated to a write operation is unique
        logWarning(DDSROUTER_SERVICEREGISTRY,
                "ServiceRegistry for service " << topic_ << " in participant " << participant_id_ <<
                " attempting to add entry with already present SequenceNumber.");
        return;
    }

    registry_[idx] = new_entry;

    // Remove oldest entry if registry full
    // TMP: Use configuration specs value when available
    if (registry_.size() == DEFAULT_MAX_ENTRIES_)
    {
        registry_.erase(registry_.begin());
    }
}

std::pair<ParticipantId, SampleIdentity> ServiceRegistry::get(
        SequenceNumber idx) const noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    std::pair<ParticipantId, SampleIdentity> ret;
    if (registry_.count(idx))
    {
        ret = registry_.at(idx);
    }
    else
    {
        ret = {ParticipantId(), SampleIdentity()};
    }

    return ret;
}

void ServiceRegistry::erase(
        SequenceNumber idx) noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (registry_.count(idx))
    {
        registry_.erase(idx);
    }
}

RpcTopic ServiceRegistry::topic() const noexcept
{
    return topic_;
}

std::recursive_mutex& ServiceRegistry::get_mutex() noexcept
{
    return mutex_;
}

} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
