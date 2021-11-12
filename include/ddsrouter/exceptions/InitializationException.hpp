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
 * @file InitializationException.hpp
 */

#ifndef _DDS_ROUTER_EXCEPTIONS_INITIALIZATIONEXCEPTION_HPP_
#define _DDS_ROUTER_EXCEPTIONS_INITIALIZATIONEXCEPTION_HPP_

#include <ddsrouter/exceptions/DDSRouterException.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * @brief TODO
 */
class InitializationException : public DDSRouterException
{
    // Use parent class constructors
    using DDSRouterException::DDSRouterException;
};

} // namespace ddsrouter
} // namespace eprosima

#endif // _DDS_ROUTER_EXCEPTIONS_INITIALIZATIONEXCEPTION_HPP_

