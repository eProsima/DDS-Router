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
 * @file SimpleParticipantConfiguration.hpp
 */

#ifndef _DDSROUTERCORE_CONFIGURATION_PARTICIPANT_SIMPLEPARTICIPANTCONFIGURATION_HPP_
#define _DDSROUTERCORE_CONFIGURATION_PARTICIPANT_SIMPLEPARTICIPANTCONFIGURATION_HPP_

#include <ddsrouter_core/configuration/participant/ParticipantConfiguration.hpp>
#include <ddsrouter_core/library/library_dll.h>
#include <ddsrouter_core/types/dds/DomainId.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace configuration {

/**
 * This dat struct represents a configuration for a SimpleParticipant
 */
struct SimpleParticipantConfiguration : public ParticipantConfiguration
{
public:

    /////////////////////////
    // CONSTRUCTORS
    /////////////////////////
    DDSROUTER_CORE_DllAPI SimpleParticipantConfiguration() = default;

    DDSROUTER_CORE_DllAPI SimpleParticipantConfiguration(
            const types::ParticipantId& id,
            const types::ParticipantKind& kind,
            const types::DomainId& domain_id) noexcept;

    /////////////////////////
    // METHODS
    /////////////////////////

    DDSROUTER_CORE_DllAPI virtual bool is_valid(
            utils::Formatter& error_msg) const noexcept override;

    /**
     * @brief Equal comparator
     *
     * @param [in] other: SimpleParticipantConfiguration to compare.
     * @return True if both configurations are the same, False otherwise.
     */
    DDSROUTER_CORE_DllAPI bool operator ==(
            const SimpleParticipantConfiguration& other) const noexcept;

    /////////////////////////
    // VARIABLES
    /////////////////////////

    types::DomainId domain = types::DomainId(0u);
};

} /* namespace configuration */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_CONFIGURATION_PARTICIPANT_SIMPLEPARTICIPANTCONFIGURATION_HPP_ */
