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
 * @file SimpleParticipant.hpp
 */

#ifndef __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_RTPS_GENERICPARTICIPANT_HPP_
#define __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_RTPS_GENERICPARTICIPANT_HPP_

#include <ddsrouter_core/configuration/participant/ParticipantConfiguration.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

class DiscoveryDatabase;

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

/**
 * Generic participant interface templated with ParticipantKind
 */
template <types::ParticipantKind PartKind, class SuperParticipantT>
class GenericParticipant : public SuperParticipantT
{
public:

    using SuperParticipantT::SuperParticipantT;

    /**
     * @brief Construct a new Participant object
     *
     * It uses the \c BaseParticipant constructor.
     * Apart from BaseParticipant, it creates a new RTPSParticipant with default Attributes and domain given
     * by configuration.
     *
     * @throw \c InitializationException in case any internal error has ocurred while creating RTPSParticipant
     * @throw \c IConfigurationException in case configuration was incorrectly set
     */
    GenericParticipant(
            const configuration::ParticipantConfiguration& participant_configuration,
            DiscoveryDatabase& discovery_database);
};

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_RTPS_GENERICPARTICIPANT_HPP_ */
