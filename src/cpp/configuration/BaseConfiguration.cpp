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
 * @file BaseConfiguration.cpp
 *
 */

#include <ddsrouter/configuration/BaseConfiguration.hpp>

namespace eprosima {
namespace ddsrouter {

BaseConfiguration::BaseConfiguration(
        const RawConfiguration& raw_configuration)
    : raw_configuration_(raw_configuration)
{
}

RawConfiguration BaseConfiguration::raw_configuration() const noexcept
{
    return raw_configuration_;
}

} /* namespace ddsrouter */
} /* namespace eprosima */
