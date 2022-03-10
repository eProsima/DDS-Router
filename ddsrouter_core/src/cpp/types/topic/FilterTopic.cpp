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
 * @file FilterTopic.cpp
 *
 */

#include <ddsrouter_core/types/topic/FilterTopic.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

FilterTopic::FilterTopic(
        const std::string& topic_name,
        const std::string& topic_type,
        bool has_keyed_set, /* = false */
        bool topic_with_key /* = false */) noexcept
    : Topic(topic_name, topic_type, topic_with_key)
    , has_keyed_set_(has_keyed_set)
{
}

bool FilterTopic::operator ==(
        const FilterTopic& other) const
{

    return Topic::operator==(other) && has_keyed_set_ == other.has_keyed_set_;
}

bool FilterTopic::has_keyed_set() const
{
    return has_keyed_set_;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
