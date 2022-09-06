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
 * @file macros.hpp
 *
 * This file contains constant values common for the whole project
 */

#ifndef _DDSROUTERUTILS_MACROS_MACROS_HPP_
#define _DDSROUTERUTILS_MACROS_MACROS_HPP_

#include <type_traits>

namespace eprosima {
namespace ddsrouter {
namespace utils {

/////////////////////////
// FORMAT
/////////////////////////

/**
 * @brief Get string of the argument passed to the macro
 *
 * @example
 * STRINGIFY(value) = "value"
 */
#define STRINGIFY(x) #x

//! Same as \c STRINGIFY but adding a comma "," at the end
#define STRINGIFY_WITH_COMMA(x) #x,


/////////////////////////
// TYPES
/////////////////////////

/**
 * @brief Force the specialization type of a template to be a subclass of a Class.
 *
 * @example
 * FORCE_TEMPLATE_SUBCLASS(A, B)  =  static assert  <=>  B not inherit from A
 *
 * @param base parent class that \c derived must inherit.
 * @param derived specialization class.
 */
#define FORCE_TEMPLATE_SUBCLASS(base, derived) \
    static_assert(std::is_base_of<base, derived>::value, STRINGIFY(derived) " class not derived from " STRINGIFY(base))

/**
 * @brief Get string of the name of the CPP Data Type of the argument
 *
 * @example
 * STRINGIFY(int) = "j"
 * STRINGIFY(string) = "NSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE"
 */
#define TYPE_NAME(x) typeid(x).name()

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERUTILS_MACROS_MACROS_HPP_ */
