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
 * @file Guid.cpp
 *
 */

#include <ddsrouter/types/endpoint/Guid.hpp>

namespace eprosima {
namespace ddsrouter {

Guid& Guid::operator = (
        const fastrtps::rtps::GUID_t& other) noexcept
{
    this->guidPrefix = other.guidPrefix;
    this->entityId = other.entityId;
    return *this;
}

bool Guid::is_valid() const noexcept
{
    return guidPrefix != eprosima::fastrtps::rtps::GuidPrefix_t::unknown() &&
           entityId != eprosima::fastrtps::rtps::EntityId_t::unknown();
}

} /* namespace ddsrouter */
} /* namespace eprosima */
