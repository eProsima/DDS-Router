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
 * @file DDSRouterException.hpp
 */

#ifndef _DDS_ROUTER_EXCEPTIONS_DDS_ROUTEREXCEPTION_HPP_
#define _DDS_ROUTER_EXCEPTIONS_DDS_ROUTEREXCEPTION_HPP_

#include <exception>
#include <string>

namespace eprosima {
namespace ddsrouter {

/**
 * @brief Base class for all exceptions thrown by the eProsima DDSRouter library.
 *
 */
class DDSRouterException : public std::exception
{

public:

    /**
     * @brief Construct a new statistics_backend::DDSRouterException object
     *
     * @param message The message to be returned by what()
     */
    DDSRouterException(
            const char* message) noexcept;

    /**
     * @brief Construct a new statistics_backend::DDSRouterException object
     *
     * @param message The message to be returned by what()
     */
    DDSRouterException(
            const std::string& message);

    /**
     * @brief Copies the statistics_backend::DDSRouterException object into a new one
     *
     * @param other The original exception object to copy
     */
    DDSRouterException(
            const DDSRouterException& other) = default;

    /**
     * @brief Copies the statistics_backend::DDSRouterException object into the current one
     *
     * @param other The original exception object to copy
     * @return the current statistics_backend::DDSRouterException object after the copy
     */
    DDSRouterException& operator =(
            const DDSRouterException& other) = default;

    /**
     * @brief Returns the explanatory string of the exception
     *
     * @return Null-terminated string with the explanatory information
     */
    virtual const char* what() const noexcept override;

protected:

    std::string message_;
};

} // namespace ddsrouter
} // namespace eprosima

#endif // _DDS_ROUTER_EXCEPTIONS_DDS_ROUTEREXCEPTION_HPP_

