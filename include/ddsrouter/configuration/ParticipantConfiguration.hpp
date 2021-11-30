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

#ifndef _DDSROUTER_CONFIGURATION_PARTICIPANTCONFIGURATION_HPP_
#define _DDSROUTER_CONFIGURATION_PARTICIPANTCONFIGURATION_HPP_

#include <ddsrouter/types/participant/ParticipantId.hpp>
#include <ddsrouter/types/participant/ParticipantType.hpp>
#include <ddsrouter/types/RawConfiguration.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * This class joins every DDSRouter Participant Configuration characteristic and includes methods to interact with it.
 * Each Participant that requires specific configuration must inherit from this class.
 */
class ParticipantConfiguration
{
public:

    /**
     * @brief Construct an invalid configuration
     *
     * The type of this configuration will be invalid and the yaml empty
     *
     * @param [in] id of the participant that will be created with this configuration
     */
    ParticipantConfiguration(
            ParticipantId id) noexcept;

    /**
     * @brief Construct a new configuration
     *
     * Yaml configuration must be a map or empty.
     * The type is set in construction. If the type is not valid, it will cause an exception.
     * The type of a participant could be set in the yaml configuration, or it could be the name of its id.
     *
     * @param [in] id of the participant that will be created with this configuration
     * @param [in] raw_configuration yaml to get the configuration
     *
     * @throw \c ConfigurationException in case the type could not be correctly set by yaml or id,
     * or if yaml is not well-formed
     */
    ParticipantConfiguration(
            ParticipantId id,
            const RawConfiguration& raw_configuration);

    //! Copy constructor
    ParticipantConfiguration(
            const ParticipantConfiguration& configuration);

    //! Participant Type associated with this configuration
    ParticipantType type() const noexcept;

    //! Participant Id associated with this configuration
    ParticipantId id() const noexcept;

    //! Yaml Raw Configuration of this configuration object
    RawConfiguration raw_configuration() const noexcept;

    /**
     * @brief Equal comparator
     *
     * This comparator should check if the id is equal to the other Configuration and check the yaml equality.
     *
     * @todo: check equality yaml and not identity yaml.
     *
     * @param [in] other: ParticipantConfiguration to compare.
     * @return True if both configurations are the same, False otherwise.
     */
    bool operator ==(
            const ParticipantConfiguration& other) const noexcept;

protected:

    /**
     * @brief Set type
     *
     * Type is set by checking type tag in yaml, and if it does not exist, check if the participant name is
     * already a type.
     */
    void set_type_() noexcept;

    //! Participant Id associated with this configuration
    const ParticipantId id_;

    //! Participant Type associated with this configuration
    ParticipantType type_;

    //! Yaml object with the configuration info
    const RawConfiguration raw_configuration_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_CONFIGURATION_PARTICIPANTCONFIGURATION_HPP_ */
