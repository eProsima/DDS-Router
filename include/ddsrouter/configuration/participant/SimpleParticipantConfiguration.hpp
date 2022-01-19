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

#ifndef _DDSROUTER_CONFIGURATION_SIMPLEPARTICIPANTCONFIGURATION_HPP_
#define _DDSROUTER_CONFIGURATION_SIMPLEPARTICIPANTCONFIGURATION_HPP_

#include <ddsrouter/configuration/participant/ParticipantConfiguration.hpp>
#include <ddsrouter/types/endpoint/DomainId.hpp>

namespace eprosima {
namespace ddsrouter {
namespace configuration {

/**
 * This class joins Simple Participant Configuration features and give methods to interact with it.
 */
class SimpleParticipantConfiguration : public ParticipantConfiguration
{
public:

    //! TODO
    SimpleParticipantConfiguration(
            const ParticipantId& id,
            const ParticipantKind& type = ParticipantKind::SIMPLE_RTPS,
            const DomainId& domain_id = DEFAULT_DOMAIN_ID_) noexcept;

    /**
     * @brief Return domain set in the configuration
     *
     * In case domain is not set in Configuration, it returns the default DomainID = 0
     *
     * @return DomainId
     */
    DomainId domain() const noexcept;

    bool operator ==(
            const SimpleParticipantConfiguration& other) const noexcept;

    virtual bool is_valid() const noexcept override;

protected:

    DomainId domain_;

    static const DomainId DEFAULT_DOMAIN_ID_; // 0
};

} /* namespace configuration */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_CONFIGURATION_SIMPLEPARTICIPANTCONFIGURATION_HPP_ */
