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

#ifndef _DDSROUTERUTILS_MACROS_HPP_
#define _DDSROUTERUTILS_MACROS_HPP_

#include <type_traits>

namespace eprosima {
namespace ddsrouter {
namespace utils {

//! Return the name of an argument called in a macro
#define STRINGIFY(x) #x

/**
 * @brief Force a template (function or class) to be specialized by a child class that inherits from a specific class.
 *
 * @param base Base class that must be specialized in template.
 * @param derived Specialization of template.
 *
 * e.g.
 * Specialization of function foo must inherit from class Foo:
 * <template class T> void foo(T t) { FORCE_TEMPLATE_SUBCLASS(Foo, T); ... };
 */
#define FORCE_TEMPLATE_SUBCLASS(base, derived) \
    static_assert(std::is_base_of<base, derived>::value, STRINGIFY(derived) " class not derived from " STRINGIFY(base))

// TODO: probably in the future is needed to create a utils method that transforms this name to a human reasonable name
//! Get the name of the class
#define TYPE_NAME(x) typeid(x).name()

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERUTILS_MACROS_HPP_ */
