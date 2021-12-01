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

#include <assert.h>

#include <ddsrouter/types/address/DiscoveryServerConnectionAddress.hpp>

namespace eprosima {
namespace ddsrouter {

DiscoveryServerConnectionAddress::DiscoveryServerConnectionAddress(
        GuidPrefix discovery_server_guid,
        std::set<Address> addresses)
    : discovery_server_guid_(discovery_server_guid)
    , addresses_(addresses)
{
}

GuidPrefix DiscoveryServerConnectionAddress::discovery_server_guid() const noexcept
{
    return discovery_server_guid_;
}

std::set<Address> DiscoveryServerConnectionAddress::addresses() const noexcept
{
    return addresses_;
}

bool DiscoveryServerConnectionAddress::is_valid() const noexcept
{
    if(!discovery_server_guid_.is_valid())
    {
        return false;
    }

    for (auto address : addresses_)
    {
        if (address.is_valid())
        {
            return true;
        }
    }

    return false;
}

} /* namespace ddsrouter */
} /* namespace eprosima */
