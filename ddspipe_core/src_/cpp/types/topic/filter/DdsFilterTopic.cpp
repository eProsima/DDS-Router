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
 * @file DdsFilterTopic.cpp
 *
 */

#include <ddspipe_core/types/topic/filter/DdsFilterTopic.hpp>
#include <cpp_utils/utils.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {
namespace types {

bool DdsFilterTopic::operator < (
        const DdsFilterTopic& other) const noexcept
{
    return utils::generic_to_string(other) < utils::generic_to_string(*this);
}

bool DdsFilterTopic::operator == (
        const DdsFilterTopic& other) const noexcept
{
    return utils::generic_to_string(other) == utils::generic_to_string(*this);
}

std::ostream& operator <<(
        std::ostream& os,
        const DdsFilterTopic& t)
{
    t.serialize(os);
    return os;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
