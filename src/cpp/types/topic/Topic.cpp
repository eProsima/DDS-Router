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

#include <ddsrouter/types/topic/Topic.hpp>

namespace eprosima {
namespace ddsrouter {

Topic::Topic(
        std::string topic_name,
        std::string topic_type) noexcept
    : topic_name_(topic_name)
    , topic_type_(topic_type)
{
}

const std::string& Topic::topic_name() const
{
    return topic_name_;
}

const std::string& Topic::topic_type() const
{
    return topic_type_;
}

bool Topic::operator ==(
        const Topic& other) const
{

    return topic_name_ == other.topic_name_ && topic_type_ == other.topic_type_;
}

bool Topic::operator <(
        const Topic& other) const
{
    int name_comparison = topic_name_.compare(other.topic_name_);
    if (name_comparison < 0)
    {
        return true;
    }
    else if (name_comparison > 0)
    {
        return false;
    }
    else
    {
        // To equal name, compare type
        // WARNING: do not return value from compare, as -1 != false
        if (topic_type_.compare(other.topic_type_) < 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}

std::ostream& operator <<(
        std::ostream& os,
        const Topic& a)
{
    os << "Topic{" << a.topic_name() << ", " << a.topic_type() << "}";
    return os;
}

} /* namespace ddsrouter */
} /* namespace eprosima */
