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

#include <communication/rpc/ServiceRegistry.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

using namespace eprosima::ddsrouter::core::types;

ServiceRegistry::ServiceRegistry(
            const RPCTopic& topic,
            const ParticipantId& server_participant_id,
            const SampleIdentity& related_sample_identity)
            : topic_(topic)
            , server_participant_id_(server_participant_id)
            , related_sample_identity_(related_sample_identity)
            , enabled_(false)
{
    // logDebug()
}

void ServiceRegistry::enable()
{
    enabled_ = true;
}

void ServiceRegistry::disable()
{
    enabled_ = false;
}

bool ServiceRegistry::enabled() const
{
    return enabled_;
}

SampleIdentity ServiceRegistry::related_sample_identity_nts()
{
    return related_sample_identity_;
}

void ServiceRegistry::add(SequenceNumber idx, std::pair<ParticipantId, SampleIdentity> new_entry)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (registry_.count(idx))
    {
        // logError
        std::cout << "Error adding entry" << std::endl;
        return;
    }

    registry_[idx] = new_entry;
}

std::pair<ParticipantId, SampleIdentity> ServiceRegistry::get(SequenceNumber idx)
{
    std::lock_guard<std::mutex> lock(mutex_);

    std::pair<ParticipantId, SampleIdentity> ret;
    if (registry_.count(idx))
    {
        ret = registry_[idx];
    }
    else
    {
        // logError
        std::cout << "Error getting entry" << std::endl;
        ret = {ParticipantId(), SampleIdentity()};
    }

    return ret;
}

void ServiceRegistry::erase(SequenceNumber idx)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (registry_.count(idx))
    {
        registry_.erase(idx);
    }
    else
    {
        // logError
        std::cout << "Error erasing entry" << std::endl;
    }
}

RPCTopic ServiceRegistry::topic()
{
    return topic_;
}

ParticipantId ServiceRegistry::server_participant_id()
{
    return server_participant_id_;
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
