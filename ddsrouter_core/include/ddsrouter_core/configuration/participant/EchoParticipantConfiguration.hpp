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
 * @file EchoParticipantConfiguration.hpp
 */

#ifndef _DDSROUTERCORE_CONFIGURATION_PARTICIPANT_ECHOPARTICIPANTCONFIGURATION_HPP_
#define _DDSROUTERCORE_CONFIGURATION_PARTICIPANT_ECHOPARTICIPANTCONFIGURATION_HPP_

#include <ddsrouter_core/configuration/participant/ParticipantConfiguration.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace configuration {

struct EchoParticipantConfiguration : public ParticipantConfiguration
{

    //! Use default parent constructors
    using ParticipantConfiguration::ParticipantConfiguration;

    /////////////////////////
    // CONSTRUCTORS
    /////////////////////////
    DDSROUTER_CORE_DllAPI EchoParticipantConfiguration() = default;

    /////////////////////////
    // VARIABLES
    /////////////////////////

    //! Whether this Participant should echo the data received
    bool echo_data = false;
    //! Whether this Participant should echo the discovery information
    bool echo_discovery = true;
    //! Whether this Participant should echo verbose information
    bool verbose = false;
};

} /* namespace configuration */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_CONFIGURATION_PARTICIPANT_ECHOPARTICIPANTCONFIGURATION_HPP_ */
