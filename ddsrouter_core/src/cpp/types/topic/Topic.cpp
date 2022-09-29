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
 * @file Topic.cpp
 *
 */

#include <ddsrouter_core/types/topic/Topic.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

Topic::Topic(
        const std::string& topic_name) noexcept
    : topic_name(topic_name)
{
}

bool Topic::is_valid(
        utils::Formatter& error_msg) const noexcept
{
    if (topic_name.empty())
    {
        error_msg << "Topic name could not be empty. ";
        return false;
    }

    return true;
}

bool Topic::operator < (
        const Topic& other) const noexcept
{
    return this->topic_name < other.topic_name;
}

bool Topic::operator == (
        const Topic& other) const noexcept
{
    return this->topic_name == other.topic_name;
}

std::ostream& operator <<(
        std::ostream& os,
        const Topic& t)
{
    os << "Topic{" << t.topic_name << "}";
    return os;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
