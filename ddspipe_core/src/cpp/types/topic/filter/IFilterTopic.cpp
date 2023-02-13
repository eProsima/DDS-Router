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

#include <ddspipe_core/types/topic/filter/IFilterTopic.hpp>
#include <cpp_utils/utils.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {
namespace types {

bool IFilterTopic::operator < (
        const IFilterTopic& other) const noexcept
{
    return utils::generic_to_string(other) < utils::generic_to_string(*this);
}

bool IFilterTopic::operator == (
        const IFilterTopic& other) const noexcept
{
    return utils::generic_to_string(other) == utils::generic_to_string(*this);
}

std::ostream& operator <<(
        std::ostream& os,
        const IFilterTopic& t)
{
    t.serialize(os);
    return os;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
