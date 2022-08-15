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
 * @file ServiceRegistry.hpp
 */

#ifndef _DDSROUTERCORE_TYPES_DDS_SERVICEREGISTRY_HPP_
#define _DDSROUTERCORE_TYPES_DDS_SERVICEREGISTRY_HPP_

#include <atomic>
#include <map>
#include <mutex>

#include <fastdds/rtps/common/SampleIdentity.h>

#include <ddsrouter_core/types/dds/Guid.hpp>
#include <ddsrouter_core/types/participant/ParticipantId.hpp>
#include <ddsrouter_core/types/topic/RPCTopic.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

using SampleIdentity = eprosima::fastrtps::rtps::SampleIdentity;

using SequenceNumber = eprosima::fastrtps::rtps::SequenceNumber_t;

/**
 * Class used to store the information associated to a service request, so its reply can be forwarded through the
 * appropiate proxy server with the proper write parameters.
 *
 * This information is stored in a map, whose insertions and deletions are protected with a mutex.
 * Insertions are performed every time a request is sent, and deletions after a reply has been received and forwarded.
 *
 * There exists a service registry per router participant.
 *
 */
class ServiceRegistry
{
public:

    /**
     * ServiceRegistry constructor by required values
     *
     * @param topic: Topic (service) of which this ServiceRegistry manages communication
     * @param participant_id: Id of participant for which this registry is created
     * @param related_sample_identity: Proxy client identifier
     *
     * @note Always created disabled. It is first enabled when a server is discovered.
     */
    ServiceRegistry(
            const types::RPCTopic& topic,
            const types::ParticipantId& participant_id,
            const SampleIdentity& related_sample_identity);

    //! Enable registry
    void enable() noexcept;

    //! Disable registry
    void disable() noexcept;

    //! Whether the registry is enabled
    bool enabled() const noexcept;

    SampleIdentity related_sample_identity_nts() const noexcept;

    //! Add entry to the registry (if key not existing)
    void add(
            SequenceNumber idx,
            std::pair<types::ParticipantId, SampleIdentity> new_entry) noexcept;

    //! Fetch entry from the registry. Returns dummy item if not present.
    std::pair<types::ParticipantId, SampleIdentity> get(
            SequenceNumber idx) const noexcept;

    //! Remove entry from the registry (if present)
    void erase(
            SequenceNumber idx) noexcept;

    //! RPCTopic getter
    types::RPCTopic topic() const noexcept;

protected:

    //! RPCTopic (service) that this ServiceRegistry manages communication
    types::RPCTopic topic_;

    //! Id of participant for which this registry is created
    types::ParticipantId participant_id_;

    //! Proxy client identifier, required for a server to set the target of replies
    SampleIdentity related_sample_identity_;

    //! Whether the registry is activated
    std::atomic<bool> enabled_;

    //! Database with an entry per received request, and the information required for forwarding replies
    std::map<SequenceNumber, std::pair<types::ParticipantId, SampleIdentity>> registry_;

    //! Mutex to protect concurrent access to \c registry_
    mutable std::mutex mutex_;
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_DDS_SERVICEREGISTRY_HPP_ */
