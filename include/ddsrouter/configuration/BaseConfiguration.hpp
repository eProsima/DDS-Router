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
 * @file BaseConfiguration.hpp
 */

#ifndef _DDSROUTER_CONFIGURATION_BASECONFIGURATION_HPP_
#define _DDSROUTER_CONFIGURATION_BASECONFIGURATION_HPP_

#include <ddsrouter/types/utils.hpp>

namespace eprosima {
namespace ddsrouter {
namespace configuration {

/**
 * This is an Interface class that force every configuration in ddsrouter to have a \c is_valid method.
 */
class BaseConfiguration
{
public:

    virtual bool is_valid(
            utils::Formatter& error_msg) const noexcept = 0;
};

} /* namespace configuration */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_CONFIGURATION_BASECONFIGURATION_HPP_ */
