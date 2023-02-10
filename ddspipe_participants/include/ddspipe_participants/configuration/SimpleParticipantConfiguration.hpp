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
// limitations under the License\.

#pragma once

#include <ddspipe_participants/configuration/ParticipantConfiguration.hpp>
#include <ddspipe_participants/library/library_dll.h>
#include <ddspipe_core/types/dds/DomainId.hpp>

namespace eprosima {
namespace ddspipe {
namespace participants {

/**
 * This data struct represents a configuration for a SimpleParticipant
 */
struct SimpleParticipantConfiguration : public ParticipantConfiguration
{
public:

    /////////////////////////
    // CONSTRUCTORS
    /////////////////////////
    DDSPIPE_PARTICIPANTS_DllAPI SimpleParticipantConfiguration() = default;

    /////////////////////////
    // METHODS
    /////////////////////////

    DDSPIPE_PARTICIPANTS_DllAPI virtual bool is_valid(
            utils::Formatter& error_msg) const noexcept override;

    /////////////////////////
    // VARIABLES
    /////////////////////////

    core::types::DomainId domain {0u};
};

} /* namespace participants */
} /* namespace ddspipe */
} /* namespace eprosima */
