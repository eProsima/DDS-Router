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

class ServiceRegistry
{
public:

    ServiceRegistry(
            const types::RPCTopic& topic,
            const types::ParticipantId& server_participant_id);

    void enable();

    void disable();

    bool enabled();

    void set_related_sample_identity_nts(const types::Guid& reply_reader_guid);

    SampleIdentity related_sample_identity_nts();

    void add(SequenceNumber idx, std::pair<types::ParticipantId, SampleIdentity> new_entry);

    std::pair<types::ParticipantId, SampleIdentity> get(SequenceNumber idx);

    void erase(SequenceNumber idx);

    types::RPCTopic topic();

    types::ParticipantId server_participant_id();

protected:

    types::RPCTopic topic_;

    types::ParticipantId server_participant_id_;

    SampleIdentity related_sample_identity_;

    bool enabled_;

    std::map<SequenceNumber, std::pair<types::ParticipantId, SampleIdentity>> registry_;

    std::mutex mutex_;
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_DDS_SERVICEREGISTRY_HPP_ */
