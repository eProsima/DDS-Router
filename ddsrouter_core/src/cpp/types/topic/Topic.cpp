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
        bool topic_with_key, /* = false */
        bool topic_reliable /* = false */) noexcept
    : topic_name_(topic_name)
    , topic_type_(topic_type)
    , topic_with_key_(topic_with_key)
    , topic_reliable_(topic_reliable)
{
}

Topic& Topic::operator =(
        const Topic& other)
{
    this->topic_name_ = other.topic_name_;
    this->topic_type_ = other.topic_type_;
    this->topic_with_key_ = other.topic_with_key_;
    this->topic_reliable_ = other.topic_reliable_;
    return *this;
}

const std::string& Topic::topic_name() const
{
    return topic_name_;
}

const std::string& Topic::topic_type() const
{
    return topic_type_;
}

bool Topic::topic_with_key() const
{
    return topic_with_key_;
}

bool Topic::topic_reliable() const
{
    return topic_reliable_;
}

bool Topic::operator ==(
        const Topic& other) const
{

    return topic_name_ == other.topic_name_
            && topic_type_ == other.topic_type_
            && topic_with_key_ == other.topic_with_key_
            && topic_reliable_ == other.topic_reliable_;
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
                // Equal key, compare reliability
                if (topic_reliable_ == other.topic_reliable_)
                {
                    return false;
                }
                else
                {
                    return !topic_reliable_;
                }
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
    std::string reliable_str = a.topic_reliable() ? "reliable" : "best_effort";
    os << "Topic{" << a.topic_name() << ";" << a.topic_type() << ";" << keyed_str << ";" << reliable_str << "}";
    return os;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
