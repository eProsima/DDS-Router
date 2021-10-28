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

#ifndef _DATABROKER_EXCEPTIONS_CONFIGURATIONEXCEPTION_HPP_
#define _DATABROKER_EXCEPTIONS_CONFIGURATIONEXCEPTION_HPP_

#include <databroker/exceptions/DatabrokerException.hpp>

namespace eprosima {
namespace databroker {

/**
 * @brief Base class for all exceptions thrown by the eProsima statistics backend library.
 *
 */
class ConfigurationException : public DatabrokerException
{
    // Use parent class constructors
    using DatabrokerException::DatabrokerException;
};

} // namespace databroker
} // namespace eprosima

#endif // _DATABROKER_EXCEPTIONS_CONFIGURATIONEXCEPTION_HPP_

