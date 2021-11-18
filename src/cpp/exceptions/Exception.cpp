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
 * @file Exception.cpp
 */

#include <ddsrouter/exceptions/Exception.hpp>

namespace eprosima {
namespace ddsrouter {

Exception::Exception(
        const char* message) noexcept
    : message_(message)
{
}

Exception::Exception(
        const std::string& message)
    : message_(message)
{
}

Exception::Exception(
        const utils::Formatter& formatter)
    : message_(formatter.to_string())
{
}

const char* Exception::what() const noexcept
{
    return message_.c_str();
}

} // namespace ddsrouter
} // namespace eprosima
