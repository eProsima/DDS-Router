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
 * @file DDSRouterReloadConfiguration.hpp
 */

#ifndef _DDSROUTERCORE_CONFIGURATION_DDSROUTERRELOADCONFIGURATION_HPP_
#define _DDSROUTERCORE_CONFIGURATION_DDSROUTERRELOADCONFIGURATION_HPP_

#include <memory>
#include <set>

#include <cpp_utils/Formatter.hpp>

#include <ddsrouter_core/configuration/BaseConfiguration.hpp>
#include <ddsrouter_core/participants/participant/configuration/ParticipantConfiguration.hpp>
#include <ddsrouter_core/library/library_dll.h>
#include <ddsrouter_core/types/topic/filter/DdsFilterTopic.hpp>
#include <ddsrouter_core/types/topic/dds/DdsTopic.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace configuration {

/**
 * This class joins every DDSRouter feature configuration and includes methods
 * to interact with this configuration.
 */
struct DDSRouterReloadConfiguration : public BaseConfiguration
{

    /////////////////////////
    // CONSTRUCTORS
    /////////////////////////

    DDSROUTER_CORE_DllAPI DDSRouterReloadConfiguration() = default;

    DDSROUTER_CORE_DllAPI DDSRouterReloadConfiguration(
            std::set<std::shared_ptr<types::DdsFilterTopic>> allowlist,
            std::set<std::shared_ptr<types::DdsFilterTopic>> blocklist,
            std::set<std::shared_ptr<types::DdsTopic>> builtin_topics);

    /////////////////////////
    // METHODS
    /////////////////////////

    DDSROUTER_CORE_DllAPI bool is_valid(
            utils::Formatter& error_msg) const noexcept override;

    /////////////////////////
    // VARIABLES
    /////////////////////////

    std::set<std::shared_ptr<types::DdsFilterTopic>> allowlist = {};

    std::set<std::shared_ptr<types::DdsFilterTopic>> blocklist = {};

    std::set<std::shared_ptr<types::DdsTopic>> builtin_topics = {};
};

} /* namespace configuration */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_CONFIGURATION_DDSROUTERRELOADCONFIGURATION_HPP_ */
