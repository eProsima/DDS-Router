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

#include <ddsrouter/types/topic/RealTopic.hpp>
#include <ddsrouter/exceptions/UnsupportedException.hpp>

namespace eprosima {
namespace ddsrouter {

const char* RealTopic::INVALID_TOPIC_NAME = "__invalid_topic_name__";
const char* RealTopic::INVALID_TOPIC_TYPE = "__invalid_topic_type_name__";

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

bool RealTopic::is_valid()
{
    // Only with one of them invalid, the topic is invalid
    return topic_name_ != INVALID_TOPIC_NAME && topic_type_ != INVALID_TOPIC_TYPE;
}

} /* namespace ddsrouter */
} /* namespace eprosima */
