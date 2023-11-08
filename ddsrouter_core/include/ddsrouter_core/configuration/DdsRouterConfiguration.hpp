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

#include <ddspipe_core/configuration/DdsPipeConfiguration.hpp>
#include <ddspipe_core/types/topic/dds/DdsTopic.hpp>

#include <ddspipe_participants/configuration/ParticipantConfiguration.hpp>
#include <ddspipe_participants/xml/XmlHandlerConfiguration.hpp>

#include <ddsrouter_core/configuration/SpecsConfiguration.hpp>
#include <ddsrouter_core/types/ParticipantKind.hpp>

#include <ddsrouter_core/library/library_dll.h>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * This data struct joins every DdsRouter feature configuration such as:
 * - DdsPipe configuration.
 * - Participant configurations.
 * - Advanced configurations.
 */
struct DdsRouterConfiguration : public ddspipe::core::IConfiguration
{

    /////////////////////////
    // CONSTRUCTORS
    /////////////////////////

    //! Default constructor
    DDSROUTER_CORE_DllAPI DdsRouterConfiguration() = default;

    /////////////////////////
    // METHODS
    /////////////////////////

    //! Override \c is_valid method.
    DDSROUTER_CORE_DllAPI bool is_valid(
            utils::Formatter& error_msg) const noexcept override;

    /////////////////////////
    // VARIABLES
    /////////////////////////

    //! Participant configurations
    std::set<
        std::pair<
            types::ParticipantKind,
            std::shared_ptr<
                ddspipe::participants::ParticipantConfiguration>>>
    participants_configurations {};

    //! DdsPipe configuration
    ddspipe::core::DdsPipeConfiguration ddspipe_configuration {};

    //! Advanced configurations
    SpecsConfiguration advanced_options {};

    //! XML Handler configuration
    ddspipe::participants::XmlHandlerConfiguration xml_configuration {};

protected:

    //! Auxiliar method to validate that class type of the participants are compatible with their kinds.
    static bool check_correct_configuration_object_(
            const std::pair<types::ParticipantKind,
            std::shared_ptr<ddspipe::participants::ParticipantConfiguration>> configuration);

};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
