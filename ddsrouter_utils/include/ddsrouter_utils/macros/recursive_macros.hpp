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

/**
 * @file recursive_macros.hpp
 *
 * This file contains constant values common for the whole project
 */

#ifndef _DDSROUTERUTILS_MACROS_RECURSIVEMACROS_HPP_
#define _DDSROUTERUTILS_MACROS_RECURSIVEMACROS_HPP_

#include <ddsrouter_utils/macros/macros.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

/////////////////////////
// COUNT ARGUMENTS
/////////////////////////

/**
 * @brief Get 11th macro.
 *
 * @note This macro is used in \c COUNT_ARGUMENT macro.
 */
#define _ELEVENTH_ARGUMENT(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, ...) \
    a11

/**
 * @brief Count number of arguments in a variadic macro with maximum 9 variables
 *
 * @note This is an auxiliary macro that encapsulates the funtionality of \c COUNT_ARGUMENTS so that macro
 * could be changed to a higher value without breaking API.
 *
 * @note \c dummy value is required to non argument calls.
 */
#define _COUNT_ARGUMENTS__UP_TO_NINE(...)  \
    _ELEVENTH_ARGUMENT(dummy, ## __VA_ARGS__, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0)

/**
 * @brief Count number of arguments in a variadic macro
 *
 * @warning This macro only allows up to 9 values.
 *
 * @note if more arguments required, change \c _ELEVENTH_ARGUMENT for a higher one.
 *
 * @example
 * COUNT_ARGUMENTS(el1, el2, el3)  =  3
 */
#define COUNT_ARGUMENTS(...) \
    _COUNT_ARGUMENTS__UP_TO_NINE(__VA_ARGS__)


/////////////////////////
// FOR EACH
/////////////////////////

/**
 * @brief These macros allow to create an iterative APPLY_MACRO_FOR_EACH loop over every argument.
 *
 * Each item of form \c _FE_N allow to evaluate the function \c ACTION for the next \c N arguments.
 */
#define _FE_1(ACTION, X) ACTION(X)
#define _FE_2(ACTION, X, ...) ACTION(X) _FE_1(ACTION, __VA_ARGS__)
#define _FE_3(ACTION, X, ...) ACTION(X) _FE_2(ACTION, __VA_ARGS__)
#define _FE_4(ACTION, X, ...) ACTION(X) _FE_3(ACTION, __VA_ARGS__)
#define _FE_5(ACTION, X, ...) ACTION(X) _FE_4(ACTION, __VA_ARGS__)
#define _FE_6(ACTION, X, ...) ACTION(X) _FE_5(ACTION, __VA_ARGS__)
#define _FE_7(ACTION, X, ...) ACTION(X) _FE_6(ACTION, __VA_ARGS__)
#define _FE_8(ACTION, X, ...) ACTION(X) _FE_7(ACTION, __VA_ARGS__)
#define _FE_9(ACTION, X, ...) ACTION(X) _FE_8(ACTION, __VA_ARGS__)
//... repeat as needed

/**
 * @brief Get the 9th argument
 *
 * @note this is useful for \c APPLY_MACRO_FOR_EACH macro.
 */
#define _GET_NINTH_ARGUMENT(_1,_2,_3,_4,_5,_6,_7,_8,_9,NAME,...) \
    NAME

#define _APPLY_MACRO_FOR_EACH__UP_TO_NINE(action,...) \
  _GET_NINTH_ARGUMENT(__VA_ARGS__,_FE_9,_FE_8,_FE_7,_FE_6,_FE_5,_FE_4,_FE_3,_FE_2,_FE_1)(action,__VA_ARGS__)

/**
 * @brief Execute \c action (must be a macro) for every argument after it.
 *
 * @warning This macro only allows up to 9 values.
 *
 * @note if more arguments required, change \c _APPLY_MACRO_FOR_EACH__UP_TO_NINE for a higher one.
 *
 * @example
 * APPLY_MACRO_FOR_EACH(print, el1, el2, el3)  =>  print(el1); print(el2); print(el3);
 */
#define APPLY_MACRO_FOR_EACH(action,...) \
    _APPLY_MACRO_FOR_EACH__UP_TO_NINE(action,__VA_ARGS__)

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERUTILS_MACROS_RECURSIVEMACROS_HPP_ */
