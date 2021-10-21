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
 * This file contains macros and shortcuts.
 */

#ifndef _DATABROKER_TYPES_MACROS_HPP_
#define _DATABROKER_TYPES_MACROS_HPP_

//! Get argument as a string
#define STRINGIFY(X) #X

/**
 * This is ussed in templated classes or functions where the data type must expand a class (Java way).
 *
 * In case this is not fulfilled it will throw an error in compilation time (static_assert).
 *
 * @param template_class data type that is going to be used.
 * @param superclass class that \c template_class must inherit.
 */
#define TEMPLATE_EXPANDS(template_class, superclass)                                                    \
    static_assert(                                                                                      \
        std::is_base_of<superclass, template_class>::value ,                                            \
        "" STRINGIFY(template_class) " template data type does not inherit from " STRINGIFY(superclass))

#endif /* _DATABROKER_TYPES_MACROS_HPP_ */
