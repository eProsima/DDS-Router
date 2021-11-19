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
 * @file ConfigurationException.hpp
 */

#ifndef _DDSROUTER_EXCEPTIONS_CONFIGURATIONEXCEPTION_HPP_
#define _DDSROUTER_EXCEPTIONS_CONFIGURATIONEXCEPTION_HPP_

#include <ddsrouter/exceptions/Exception.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * @brief Exception thrown when there has been some error reading a configuration file, reading a corrupted or
 *  bad formed yaml and when a configuration creation has failed.
 */
class ConfigurationException : public Exception
{
    // Use parent class constructors
    using Exception::Exception;
};

} // namespace ddsrouter
} // namespace eprosima

#endif // _DDSROUTER_EXCEPTIONS_CONFIGURATIONEXCEPTION_HPP_

