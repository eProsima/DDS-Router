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

#include <ddsrouter_utils/Log.hpp>

#include <communication/rpc/ServiceRegistry.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

using namespace eprosima::ddsrouter::core::types;

ServiceRegistry::ServiceRegistry(
        const RPCTopic& topic,
        const ParticipantId& participant_id,
        const SampleIdentity& related_sample_identity)
    : topic_(topic)
    , participant_id_(participant_id)
    , related_sample_identity_(related_sample_identity)
    , enabled_(false)
{
    logDebug(DDSROUTER_SERVICEREGISTRY,
            "ServiceRegistry for service " << topic <<
            " created with related_sample_identity " << related_sample_identity <<
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

SampleIdentity ServiceRegistry::related_sample_identity_nts() const noexcept
{
    return related_sample_identity_;
}

void ServiceRegistry::add(
        SequenceNumber idx,
        std::pair<ParticipantId, SampleIdentity> new_entry) noexcept
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (registry_.count(idx))
    {
        // Should never occur as each sequence number associated to a write operation is unique
        logWarning(DDSROUTER_SERVICEREGISTRY,
                "ServiceRegistry for service " << topic_ << " in participant " << participant_id_ <<
                " attempting to add entry with already present SequenceNumber.");
        return;
    }
    logDebug(DDSROUTER_SERVICEREGISTRY,
                "ServiceRegistry for service " << topic_ << " in participant " << participant_id_ <<
                " adding entry with SequenceNumber " << idx);
    registry_[idx] = new_entry;
}

std::pair<ParticipantId, SampleIdentity> ServiceRegistry::get(
        SequenceNumber idx) const noexcept
{
    std::lock_guard<std::mutex> lock(mutex_);

    logDebug(DDSROUTER_SERVICEREGISTRY,
                "ServiceRegistry for service " << topic_ << " in participant " << participant_id_ <<
                " attempting to get entry with SequenceNumber " << idx);

    std::pair<ParticipantId, SampleIdentity> ret;
    if (registry_.count(idx))
    {
        ret = registry_.at(idx);
    }
    else
    {
        logError(DDSROUTER_SERVICEREGISTRY,
                "ServiceRegistry for service " << topic_ << " in participant " << participant_id_ <<
                " FAILED TO GET entry with SequenceNumber " << idx);
        ret = {ParticipantId(), SampleIdentity()};
    }

    return ret;
}

void ServiceRegistry::erase(
        SequenceNumber idx) noexcept
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (registry_.count(idx))
    {
        logDebug(DDSROUTER_SERVICEREGISTRY,
                "ServiceRegistry for service " << topic_ << " in participant " << participant_id_ <<
                " erasing entry with SequenceNumber " << idx);
        registry_.erase(idx);
    }
    else
    {
        logError(DDSROUTER_SERVICEREGISTRY,
                "ServiceRegistry for service " << topic_ << " in participant " << participant_id_ <<
                " FAILED TO ERASE entry with SequenceNumber " << idx);
    }
}

RPCTopic ServiceRegistry::topic() const noexcept
{
    return topic_;
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
