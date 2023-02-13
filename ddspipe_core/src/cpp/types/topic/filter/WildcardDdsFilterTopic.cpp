// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <cpp_utils/types/cast.hpp>
#include <cpp_utils/utils.hpp>

#include <ddspipe_core/types/topic/filter/WildcardDdsFilterTopic.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {
namespace types {

bool WildcardDdsFilterTopic::contains(
        const IFilterTopic& other) const
{
    if (!IS_SAME_TYPE_AS_THIS(other))
    {
        return false;
    }
    // TODO: implement
    static_cast<void> (other);
    return false;
}

bool WildcardDdsFilterTopic::matches(
        const ITopic& other) const
{
    if (utils::can_cast<DdsTopic>(other))
    {
        return matches_(static_cast<const DdsTopic&>(other));
    }
    return false;
}

/////////////////////////
// SERIALIZATION METHODS
/////////////////////////

std::ostream& WildcardDdsFilterTopic::serialize(
        std::ostream& os) const
{
    os << *this;
    return os;
}

std::ostream& operator <<(
        std::ostream& os,
        const WildcardDdsFilterTopic& t)
{
    os << "WildcardDdsFilterTopic{" << t.topic_name << ";" << t.type_name << "}";
    return os;
}

bool WildcardDdsFilterTopic::matches_(
        const DdsTopic& real_topic) const
{
    // Compare topic_name
    if (this->topic_name.is_set())
    {
        if (!utils::match_pattern(this->topic_name.get_reference(), real_topic.topic_name()))
        {
            return false;
        }
    }

    // Compare type_name
    if (this->type_name.is_set())
    {
        if (!utils::match_pattern(this->type_name.get_reference(), real_topic.type_name))
        {
            return false;
        }
    }

    return true;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
