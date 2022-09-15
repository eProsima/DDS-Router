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

#include <ddsrouter_core/types/dds/Guid.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

Guid::Guid(
        const fastrtps::rtps::GUID_t& x)
{
    guidPrefix = x.guidPrefix;
    entityId = x.entityId;
}

Guid::Guid(
        fastrtps::rtps::GUID_t&& x)
{
    guidPrefix = std::move(x.guidPrefix);
    entityId = std::move(x.entityId);
}

Guid& Guid::operator = (
        const fastrtps::rtps::GUID_t& other) noexcept
{
    this->guidPrefix = other.guidPrefix;
    this->entityId = other.entityId;
    return *this;
}

bool Guid::is_valid() const noexcept
{
    return guid_prefix().is_valid() &&
           entityId != eprosima::fastrtps::rtps::EntityId_t::unknown();
}

GuidPrefix Guid::guid_prefix() const noexcept
{
    return GuidPrefix(guidPrefix);
}

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
