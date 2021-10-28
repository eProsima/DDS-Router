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
 * @file DatabrokerException.hpp
 */

#ifndef _DATABROKER_EXCEPTIONS_DATABROKEREXCEPTION_HPP_
#define _DATABROKER_EXCEPTIONS_DATABROKEREXCEPTION_HPP_

#include <exception>
#include <string>

namespace eprosima {
namespace databroker {

/**
 * @brief Base class for all exceptions thrown by the eProsima statistics backend library.
 *
 */
class DatabrokerException : public std::exception
{

public:

    /**
     * @brief Construct a new statistics_backend::DatabrokerException object
     *
     * @param message The message to be returned by what()
     */
    DatabrokerException(
            const char* message) noexcept;

    /**
     * @brief Construct a new statistics_backend::DatabrokerException object
     *
     * @param message The message to be returned by what()
     */
    DatabrokerException(
            const std::string& message);

    /**
     * @brief Copies the statistics_backend::DatabrokerException object into a new one
     *
     * @param other The original exception object to copy
     */
    DatabrokerException(
            const DatabrokerException& other) = default;

    /**
     * @brief Copies the statistics_backend::DatabrokerException object into the current one
     *
     * @param other The original exception object to copy
     * @return the current statistics_backend::DatabrokerException object after the copy
     */
    DatabrokerException& operator =(
            const DatabrokerException& other) = default;

    /**
     * @brief Returns the explanatory string of the exception
     *
     * @return Null-terminated string with the explanatory information
     */
    virtual const char* what() const noexcept override;

protected:

    std::string message_;
};

} // namespace databroker
} // namespace eprosima

#endif // _DATABROKER_EXCEPTIONS_DATABROKEREXCEPTION_HPP_

