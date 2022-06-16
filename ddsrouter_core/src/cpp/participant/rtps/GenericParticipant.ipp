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
 * @file LocalDiscoveryServerParticipant.cpp
 */
#ifndef __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_RTPS_GENERICPARTICIPANT_IPP_
#define __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_RTPS_GENERICPARTICIPANT_IPP_

#include <dynamic/DiscoveryDatabase.hpp>
#include <participant/rtps/GenericParticipant.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

template <types::ParticipantKind PartKind, class SuperParticipantT>
GenericParticipant<PartKind, SuperParticipantT>::GenericParticipant(
        const configuration::ParticipantConfiguration& participant_configuration,
        DiscoveryDatabase& discovery_database)
    : SuperParticipantT(participant_configuration, discovery_database)
{
    SuperParticipantT::create_participant_();
}

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_RTPS_GENERICPARTICIPANT_IPP_ */
