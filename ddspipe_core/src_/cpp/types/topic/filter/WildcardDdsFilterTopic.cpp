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

/**
 * @file WildcardDdsFilterTopic.cpp
 *
 */

#include <ddspipe_core/types/topic/filter/WildcardDdsFilterTopic.hpp>
#include <cpp_utils/utils.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {
namespace types {

WildcardDdsFilterTopic::WildcardDdsFilterTopic(
        const std::string topic_name /* = "*" */)
    : topic_name(topic_name)
{
}

bool WildcardDdsFilterTopic::contains(
        const DdsFilterTopic& other) const
{
    // TODO: implement
    static_cast<void> (other);
    return false;
}

bool WildcardDdsFilterTopic::matches(
        const DistributedTopic& other) const
{
    // Compare key
    if (this->keyed.is_set())
    {
        if (this->keyed != other.keyed)
        {
            return false;
        }
    }

    // Compare type_name
    if (this->type_name.is_set())
    {
        if (!utils::match_pattern(this->type_name.get_reference(), other.type_name))
        {
            return false;
        }
    }

    // Compare topic name
    return utils::match_pattern(this->topic_name, other.topic_name);
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
    std::string keyed_str = t.keyed.is_set()
        ? STR_ENTRY << ";keyed"
        : STR_ENTRY << "";

    std::string type_name_str = t.type_name.is_set()
        ? STR_ENTRY << ";" << t.type_name
        : STR_ENTRY << "";

    os << "WildcardDdsFilterTopic{" << t.topic_name << type_name_str << keyed_str << "}";
    return os;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
