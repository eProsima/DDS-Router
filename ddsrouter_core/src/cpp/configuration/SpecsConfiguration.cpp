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
 * @file SpecsConfiguration.cpp
 *
 */

#include <cpp_utils/Formatter.hpp>
#include <cpp_utils/Log.hpp>

#include <ddsrouter_core/configuration/SpecsConfiguration.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

bool SpecsConfiguration::is_valid(
        utils::Formatter& error_msg) const noexcept
{
    if (number_of_threads < 1)
    {
        error_msg << "Number of Threads must be at least 1.";
        return false;
    }

    if (topic_qos.history_depth == 0U)
    {
        logWarning(DDSROUTER_SPECS, "Using non limited histories could lead to memory exhaustion in long executions.");
    }

    return true;
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
