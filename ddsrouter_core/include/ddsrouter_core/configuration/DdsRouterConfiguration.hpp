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

#pragma once

#include <memory>
#include <set>

#include <ddspipe_core/types/topic/dds/DdsTopic.hpp>

#include <ddspipe_participants/configuration/ParticipantConfiguration.hpp>

#include <ddsrouter_core/library/library_dll.h>
#include <ddsrouter_core/configuration/SpecsConfiguration.hpp>
#include <ddsrouter_core/configuration/DdsRouterReloadConfiguration.hpp>
#include <ddsrouter_core/types/ParticipantKind.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * This data struct joins every DdsRouter feature configuration such as:
 * - Modifiable values (from \c DdsRouterReloadConfiguration ).
 * - Participant configurations.
 * - Advanced configurations.
 */
struct DdsRouterConfiguration : public DdsRouterReloadConfiguration
{

    /////////////////////////
    // CONSTRUCTORS
    /////////////////////////

    //! Default constructor
    DDSROUTER_CORE_DllAPI DdsRouterConfiguration() = default;

    /////////////////////////
    // METHODS
    /////////////////////////

    //! Set internal values with the values reloaded
    DDSROUTER_CORE_DllAPI void reload(
            const DdsRouterReloadConfiguration& new_configuration);

    //! Override \c is_valid method.
    DDSROUTER_CORE_DllAPI bool is_valid(
            utils::Formatter& error_msg) const noexcept override;

    /////////////////////////
    // VARIABLES
    /////////////////////////

    //! Builtin topics to create at the beggining of the execution
    std::set<utils::Heritable<ddspipe::core::types::DistributedTopic>> builtin_topics {};

    //! Participant configurations
    std::set<
        std::pair<
            types::ParticipantKind,
            std::shared_ptr<
                ddspipe::participants::ParticipantConfiguration>>>
    participants_configurations {};

    //! Advanced configurations
    SpecsConfiguration advanced_options {};

protected:

    //! Auxiliar method to validate that class type of the participants are compatible with their kinds.
    static bool check_correct_configuration_object_(
            const std::pair<types::ParticipantKind,
            std::shared_ptr<ddspipe::participants::ParticipantConfiguration>> configuration);

};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
