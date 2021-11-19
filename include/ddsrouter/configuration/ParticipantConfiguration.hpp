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
 * TODO
 */
class ParticipantConfiguration
{
public:

    ParticipantConfiguration(
            ParticipantId id,
            const RawConfiguration& raw_configuration);

    virtual ~ParticipantConfiguration();

    ParticipantType type() const;

    ParticipantId id() const;

    bool operator ==(
            const ParticipantConfiguration& other) const;

protected:

    void set_type_();

    const ParticipantId id_;

    ParticipantType type_;

    const RawConfiguration raw_configuration_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_CONFIGURATION_PARTICIPANTCONFIGURATION_HPP_ */
