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

#include <ddsrouter/configuration/ParticipantConfiguration.hpp>
#include <ddsrouter/types/dds_types.hpp>

namespace eprosima {
namespace ddsrouter {
namespace rtps {

/**
 * This Configuration give methods for a \c SimpleParticipant to be configured.
 */
class SimpleParticipantConfiguration : public ParticipantConfiguration
{
public:

    //! Using parent constructors
    using ParticipantConfiguration::ParticipantConfiguration;

    //! Copy constructor from superclass. Needed by \c ParticipantFactory .
    SimpleParticipantConfiguration(
            const ParticipantConfiguration& configuration);

    /**
     * @brief Return domain set in the configuration
     *
     * In case domain is not set in Configuration, it returns the default DomainID = 0
     *
     * @return DomainId
     */
    DomainId domain() const noexcept;
};

} /* namespace rtps */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_CONFIGURATION_SIMPLEPARTICIPANTCONFIGURATION_HPP_ */
