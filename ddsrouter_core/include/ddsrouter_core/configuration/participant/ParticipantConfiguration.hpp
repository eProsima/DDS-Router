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
 * @file ParticipantConfiguration.hpp
 */

#ifndef _DDSROUTERCORE_CONFIGURATION_PARTICIPANT_PARTICIPANTCONFIGURATION_HPP_
#define _DDSROUTERCORE_CONFIGURATION_PARTICIPANT_PARTICIPANTCONFIGURATION_HPP_

#include <ddsrouter_core/library/library_dll.h>
#include <ddsrouter_core/types/participant/ParticipantId.hpp>
#include <ddsrouter_utils/Formatter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace configuration {

/**
 * Base class for participants configurations
 */
class ParticipantConfiguration
{
public:

    /**
     * @brief Single constructor
     *
     * @param id Participant ID
     */
    DDSROUTER_CORE_DllAPI ParticipantConfiguration(
            const types::ParticipantId& id);

    virtual ~ParticipantConfiguration();

    /**
     * @brief Return Participant getter
     *
     * @return Const reference of participant id
     */
    DDSROUTER_CORE_DllAPI const types::ParticipantId& id() const noexcept;

protected:

    //! Owning Participant Id associated with this configuration
    const types::ParticipantId id_;
};

} /* namespace configuration */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_CONFIGURATION_PARTICIPANT_PARTICIPANTCONFIGURATION_HPP_ */
