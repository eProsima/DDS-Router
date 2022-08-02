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
 * @file RealTopic.cpp
 *
 */

#include <string>
#include <vector>

#include <ddsrouter_core/types/topic/RealTopic.hpp>
#include <ddsrouter_utils/exception/UnsupportedException.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

const char* RealTopic::INVALID_TOPIC_NAME = "__invalid_topic_name__";
const char* RealTopic::INVALID_TOPIC_TYPE = "__invalid_topic_type_name__";

RealTopic::RealTopic(
        const std::string& topic_name,
        const std::string& topic_type,
        bool topic_with_key, /* = false */
        bool topic_reliable /* = false */) noexcept
    : Topic(topic_name, topic_type, topic_with_key)
    , topic_reliable_(topic_reliable)
{
}

RealTopic::RealTopic(
        bool topic_reliable,
        const std::string& topic_name,
        const std::string& topic_type) noexcept
    : Topic(topic_name, topic_type)
    , topic_reliable_(topic_reliable)
{
}

RealTopic::RealTopic()
    : Topic(INVALID_TOPIC_NAME, INVALID_TOPIC_TYPE)
{
}

bool RealTopic::is_real_topic(
        const std::string& topic_name,
        const std::string& type_name) noexcept
{
    // Real topics must have topic name and topic type
    if (topic_name.empty() || type_name.empty())
    {
        return false;
    }

    // TODO: extend with regex
    // It checks if topic name or type contain an invalid substring
    std::vector<std::string> invalid_substrings = {
        "*", // Wildcard char
    };

    for (std::string invalid_substring : invalid_substrings)
    {
        if (topic_name.find(invalid_substring) != std::string::npos ||
                type_name.find(invalid_substring) != std::string::npos)
        {
            return false;
        }
    }
    return true;
}

bool RealTopic::is_valid() const noexcept
{
    // Only with one of them invalid, the topic is invalid
    return topic_name_ != INVALID_TOPIC_NAME &&
           topic_type_ != INVALID_TOPIC_TYPE &&
           is_real_topic(topic_name_, topic_type_);
}

bool RealTopic::topic_reliable() const
{
    return topic_reliable_;
}

void RealTopic::topic_reliable(bool new_val)
{
    topic_reliable_ = new_val;
}

std::ostream& operator <<(
        std::ostream& os,
        const RealTopic& a)
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
