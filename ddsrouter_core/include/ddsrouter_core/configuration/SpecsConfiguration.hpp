// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <ddspipe_core/configuration/IConfiguration.hpp>
#include <ddspipe_core/types/dds/TopicQoS.hpp>

#include <ddsrouter_core/library/library_dll.h>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * This data struct contains the values for advance configuration of the DDS Router such as:
 * - Number of threads to Thread Pool
 * - Default maximum history depth
 */
struct SpecsConfiguration : public ddspipe::core::IConfiguration
{

    /////////////////////////
    // CONSTRUCTORS
    /////////////////////////

    DDSROUTER_CORE_DllAPI SpecsConfiguration() = default;

    /////////////////////////
    // METHODS
    /////////////////////////

    DDSROUTER_CORE_DllAPI virtual bool is_valid(
            utils::Formatter& error_msg) const noexcept override;

    /////////////////////////
    // VARIABLES
    /////////////////////////

    unsigned int number_of_threads = 12;

    /**
     * @brief Maximum of History depth by default in those topics where it is not specified.
     *
     * @note Default value is 5000 as in Fast DDS.
     */
    ddspipe::core::types::HistoryDepthType max_history_depth = 5000;
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
