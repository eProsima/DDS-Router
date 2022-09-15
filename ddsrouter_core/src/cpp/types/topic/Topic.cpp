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
        std::string topic_name,
        std::string topic_type,
        bool topic_with_key /* = false */) noexcept
    : topic_name_(topic_name)
    , topic_type_(topic_type)
    , topic_with_key_(topic_with_key)
{
}

Topic& Topic::operator =(
        const Topic& other)
{
    this->topic_name_ = other.topic_name_;
    this->topic_type_ = other.topic_type_;
    this->topic_with_key_ = other.topic_with_key_;
    return *this;
}

const std::string& Topic::topic_name() const
{
    return topic_name_;
}

void Topic::topic_name(
        const std::string& topic_name)
{
    topic_name_ = topic_name;
}

const std::string& Topic::topic_type() const
{
    return topic_type_;
}

void Topic::topic_type(
        const std::string& topic_type)
{
    topic_type_ = topic_type;
}

bool Topic::topic_with_key() const
{
    return topic_with_key_;
}

bool Topic::operator ==(
        const Topic& other) const
{

    return topic_name_ == other.topic_name_
           && topic_type_ == other.topic_type_
           && topic_with_key_ == other.topic_with_key_;
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
        // Equal name, compare type
        // WARNING: do not return value from compare, as -1 != false
        int topic_comparison = topic_type_.compare(other.topic_type_);
        if (topic_comparison < 0)
        {
            return true;
        }
        else if (topic_comparison > 0)
        {
            return false;
        }
        else
        {
            // Equal type, compare keyed
            if (topic_with_key_ == other.topic_with_key_)
            {
                return false;
            }
            else
            {
                return !topic_with_key_;
            }
        }
    }
}

bool Topic::is_valid() const noexcept
{
    return true;
}

std::ostream& operator <<(
        std::ostream& os,
        const Topic& a)
{
    std::string keyed_str = a.topic_with_key() ? "keyed" : "no_key";
    os << "Topic{" << a.topic_name() << ";" << a.topic_type() << ";" << keyed_str << "}";
    return os;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
