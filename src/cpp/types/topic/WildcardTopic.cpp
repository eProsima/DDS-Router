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
 * @file WildcardTopic.cpp
 *
 */

#include <ddsrouter/types/topic/WildcardTopic.hpp>
#include <ddsrouter/types/utils.hpp>

namespace eprosima {
namespace ddsrouter {

WildcardTopic::WildcardTopic(
        const std::string& topic_name)
    : FilterTopic(topic_name, "*")
{
}

bool WildcardTopic::contains(
        const FilterTopic& other) const
{
    // TODO: implement
    static_cast<void> (other);
    return false;
}

bool WildcardTopic::matches(
        const RealTopic& other) const
{
    if (utils::match_pattern(this->topic_name(), other.topic_name()))
    {
        // Topic name mathes, check if type matches
        return utils::match_pattern(this->topic_type(), other.topic_type());
    }
    return false;
}

} /* namespace ddsrouter */
} /* namespace eprosima */
