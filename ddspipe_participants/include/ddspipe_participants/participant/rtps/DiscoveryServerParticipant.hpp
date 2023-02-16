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

#pragma once

#include <fastdds/rtps/transport/TCPTransportDescriptor.h>

#include <ddspipe_participants/configuration/DiscoveryServerParticipantConfiguration.hpp>
#include <ddspipe_participants/types/security/tls/TlsConfiguration.hpp>
#include <ddspipe_participants/participant/rtps/CommonParticipant.hpp>

namespace eprosima {
namespace ddspipe {
namespace participants {
namespace rtps {

/**
 * TODO
 */
class DiscoveryServerParticipant
    : public CommonParticipant
{
public:

    DDSPIPE_PARTICIPANTS_DllAPI DiscoveryServerParticipant(
            const std::shared_ptr<DiscoveryServerParticipantConfiguration>& participant_configuration,
            const std::shared_ptr<core::PayloadPool>& payload_pool,
            const std::shared_ptr<core::DiscoveryDatabase>& discovery_database);

protected:

    static fastrtps::rtps::RTPSParticipantAttributes reckon_participant_attributes_(
            const DiscoveryServerParticipantConfiguration* participant_configuration);

};

} /* namespace rpts */
} /* namespace participants */
} /* namespace ddspipe */
} /* namespace eprosima */
