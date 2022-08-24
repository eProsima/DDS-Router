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
 * @file Formatter.cpp
 *
 */

#include <ddsrouter_utils/Formatter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

Formatter::operator std::string () const noexcept
{
    return this->to_string();
}

std::string Formatter::to_string() const noexcept
{
    return ss_.str();
}

std::ostream& operator <<(
        std::ostream& os,
        const Formatter& f)
{
    os << f.to_string();
    return os;
}

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */
