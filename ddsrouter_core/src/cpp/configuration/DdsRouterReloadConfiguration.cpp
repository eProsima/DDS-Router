// Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file DdsRouterReloadConfiguration.cpp
 *
 */

#include <memory>

#include <cpp_utils/Formatter.hpp>
#include <cpp_utils/memory/Heritable.hpp>
#include <cpp_utils/Log.hpp>

#include <ddspipe_core/types/topic/filter/WildcardDdsFilterTopic.hpp>

#include <ddsrouter_core/configuration/DdsRouterReloadConfiguration.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

bool DdsRouterReloadConfiguration::is_valid(
        utils::Formatter& error_msg) const noexcept
{
    // utils::Heritable objects cannot initialize its internal pointer to nullptr.
    // Therfore, the previous comparison that checked that the topic (shared_ptr) is not nullptr does not apply anymore.
    return true;
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
