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
 * @file DiscoveryServerConnectionAddress.cpp
 *
 */

#include <ddsrouter_core/types/address/DiscoveryServerConnectionAddress.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

DiscoveryServerConnectionAddress::DiscoveryServerConnectionAddress(
        GuidPrefix discovery_server_guid,
        std::set<Address> addresses)
    : discovery_server_guid_prefix_(discovery_server_guid)
    , addresses_(addresses)
{
}

GuidPrefix DiscoveryServerConnectionAddress::discovery_server_guid_prefix() const noexcept
{
    return discovery_server_guid_prefix_;
}

const std::set<Address>& DiscoveryServerConnectionAddress::addresses() const noexcept
{
    return addresses_;
}

bool DiscoveryServerConnectionAddress::operator <(
        const DiscoveryServerConnectionAddress& other) const noexcept
{
    if (this->discovery_server_guid_prefix() == other.discovery_server_guid_prefix())
    {
        // Same Guid
        return this->addresses() < other.addresses();
    }
    else
    {
        // Different guid
        return this->discovery_server_guid_prefix() < other.discovery_server_guid_prefix();
    }
}

bool DiscoveryServerConnectionAddress::operator ==(
        const DiscoveryServerConnectionAddress& other) const noexcept
{
    return (this->discovery_server_guid_prefix() == other.discovery_server_guid_prefix()) &&
           (this->addresses() == other.addresses());
}

std::ostream& operator <<(
        std::ostream& output,
        const DiscoveryServerConnectionAddress& address)
{
    output << "{{" << address.discovery_server_guid_prefix() << "}[";
    for (auto a : address.addresses())
    {
        output << a << ",";
    }
    output << "]}";
    return output;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
