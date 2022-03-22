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
 * This class joins Simple Participant Configuration features and give methods to interact with it.
 */
class SimpleParticipantConfiguration : public ParticipantConfiguration
{
public:

    //! TODO
    DDSROUTER_CORE_DllAPI SimpleParticipantConfiguration(
            const types::ParticipantId& id,
            const types::ParticipantKind& kind = types::ParticipantKind::SIMPLE_RTPS,
            const types::DomainId& domain_id = DEFAULT_DOMAIN_ID_) noexcept;

    /**
     * @brief Return domain set in the configuration
     *
     * In case domain is not set in Configuration, it returns the default DomainID = 0
     *
     * @return DomainId
     */
    DDSROUTER_CORE_DllAPI types::DomainId domain() const noexcept;

    DDSROUTER_CORE_DllAPI bool operator ==(
            const SimpleParticipantConfiguration& other) const noexcept;

    DDSROUTER_CORE_DllAPI virtual bool is_valid(
            utils::Formatter& error_msg) const noexcept override;

protected:

    types::DomainId domain_;

    DDSROUTER_CORE_DllAPI static const types::DomainId DEFAULT_DOMAIN_ID_; // 0
};

} /* namespace configuration */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_CONFIGURATION_PARTICIPANT_SIMPLEPARTICIPANTCONFIGURATION_HPP_ */
