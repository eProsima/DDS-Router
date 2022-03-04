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
 * @file ValueNotAllowedException.hpp
 */

#ifndef _DDSROUTERUTILS_EXCEPTIONS_VALUENOTALLOWEDEXCEPTION_HPP_
#define _DDSROUTERUTILS_EXCEPTIONS_VALUENOTALLOWEDEXCEPTION_HPP_

#include <ddsrouter_utils/exception/Exception.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

/**
 * @brief Exception thrown when not allowed value is detected.
 *
 * Most common case is when an enumeration value has a value not contemplated in the enum definition.
 */
class DDSROUTERUTILS_DllAPI ValueNotAllowedException : public Exception
{
    // Use parent class constructors
    using Exception::Exception;
};

} // namespace utils
} // namespace ddsrouter
} // namespace eprosima

#endif // _DDSROUTERUTILS_EXCEPTIONS_VALUENOTALLOWEDEXCEPTION_HPP_

