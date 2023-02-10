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

#include <cpp_utils/Formatter.hpp>

#include <ddspipe_core/types/topic/filter/WildcardDdsFilterTopic.hpp>
#include <ddspipe_core/configuration/BaseConfiguration.hpp>

#include <ddsrouter_core/library/library_dll.h>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * This class joins every DdsRouter feature configuration and includes methods
 * to interact with this configuration.
 */
struct DdsRouterReloadConfiguration : public ddspipe::core::BaseConfiguration
{

    /////////////////////////
    // CONSTRUCTORS
    /////////////////////////

    DDSROUTER_CORE_DllAPI DdsRouterReloadConfiguration() = default;

    /////////////////////////
    // METHODS
    /////////////////////////

    DDSROUTER_CORE_DllAPI bool is_valid(
            utils::Formatter& error_msg) const noexcept override;

    /////////////////////////
    // VARIABLES
    /////////////////////////

    std::set<std::shared_ptr<ddspipe::core::types::WildcardDdsFilterTopic>> allowlist {};

    std::set<std::shared_ptr<ddspipe::core::types::WildcardDdsFilterTopic>> blocklist {};

};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
