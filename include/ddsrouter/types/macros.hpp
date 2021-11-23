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
 * @file constants.hpp
 *
 * This file contains constant values common for the whole project
 */

#ifndef _DDSROUTER_TYPES_MACROS_HPP_
#define _DDSROUTER_TYPES_MACROS_HPP_

#include <type_traits>

namespace eprosima {
namespace ddsrouter {

#define STRINGIFY(x) #x

#define FORCE_TEMPLATE_SUBCLASS(base, derived) \
    static_assert(std::is_base_of<base, derived>::value, STRINGIFY(derived) " class not derived from " STRINGIFY(base))

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TYPES_MACROS_HPP_ */
