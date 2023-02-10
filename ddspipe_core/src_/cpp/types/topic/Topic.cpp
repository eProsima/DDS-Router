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

#include <ddspipe_core/types/topic/Topic.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {
namespace types {

Topic::Topic(
        const std::string& topic_name,
        const TopicInternalTypeDiscriminator& internal_type_discriminator)
    : topic_name(topic_name)
    , internal_type_discriminator(internal_type_discriminator)
{
    // Do nothing
}

bool Topic::is_valid(
        utils::Formatter& error_msg) const noexcept
{
    if (topic_name.empty())
    {
        error_msg << "Topic name could not be empty. ";
        return false;
    }

    if (internal_type_discriminator == INTERNAL_TOPIC_TYPE_NONE)
    {
        error_msg << "Internal type discriminator could not be None. ";
        return false;
    }

    return true;
}

std::ostream& operator <<(
        std::ostream& os,
        const Topic& t)
{
    os << "Topic{" << t.topic_name << "," << t.internal_type_discriminator << "}";
    return os;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
