// Copyright 2024 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <cpp_utils/time/time_utils.hpp>

#include <ddspipe_core/configuration/CommandlineArgs.hpp>
#include <ddsrouter_yaml/library/library_dll.h>

namespace eprosima {
namespace ddsrouter {
namespace yaml {

/*
 * Struct to parse the executable arguments
 */
struct DDSROUTER_YAML_DllAPI CommandlineArgsRouter: public ddspipe::core::CommandlineArgs
{

    /////////////////////////
    // CONSTRUCTORS
    /////////////////////////

    CommandlineArgsRouter();

    /////////////////////////
    // METHODS
    /////////////////////////

    /**
     * @brief \c is_valid method.
     */
    bool is_valid(
            utils::Formatter& error_msg) const noexcept override;

    /////////////////////////
    // VARIABLES
    /////////////////////////

    // Maximum timeout
    utils::Duration_ms timeout{0};

};

} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */
