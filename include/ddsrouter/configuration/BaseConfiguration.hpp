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
 * @file BaseConfiguration.hpp
 */

#ifndef _DDSROUTER_CONFIGURATION_BASECONFIGURATION_HPP_
#define _DDSROUTER_CONFIGURATION_BASECONFIGURATION_HPP_

#include <ddsrouter/types/participant/ParticipantId.hpp>
#include <ddsrouter/types/participant/ParticipantKind.hpp>
#include <ddsrouter/types/RawConfiguration.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * TODO
 */
class BaseConfiguration
{
public:

    /**
     * @brief Construct a new configuration
     *
     * @param [in] raw_configuration yaml to get the configuration
     *
     * @throw \c ConfigurationException in case the configuration is not a well-formed yaml
     */
    BaseConfiguration(
            const RawConfiguration& raw_configuration);

    RawConfiguration raw_configuration() const noexcept;

protected:

    RawConfiguration raw_configuration_;

};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_CONFIGURATION_BASECONFIGURATION_HPP_ */
