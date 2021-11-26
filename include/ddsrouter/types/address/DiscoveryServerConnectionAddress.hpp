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
// See the License for the specific language governing permissions and
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// limitations under the License.

/**
 * @file DiscoveryServerConnectionAddress.hpp
 *
 */

#ifndef _DDSROUTER_TYPES_ADDRESS_DISCOVERYSERVERCONNECTIONADDRESS_HPP_
#define _DDSROUTER_TYPES_ADDRESS_DISCOVERYSERVERCONNECTIONADDRESS_HPP_

#include <string>
#include <vector>

#include <ddsrouter/types/address/Address.hpp>
#include <ddsrouter/types/endpoint/Guid.hpp>

namespace eprosima {
namespace ddsrouter {

// TODO: add comments
class DiscoveryServerConnectionAddress
{
public:

    DiscoveryServerConnectionAddress(
        GuidPrefix discovery_server_guid_,
        std::set<Address> addresses_);

    GuidPrefix discovery_server_guid() const noexcept;

    std::set<Address> addresses() const noexcept;

    virtual bool is_valid() const noexcept;

protected:

    GuidPrefix discovery_server_guid_;

    std::set<Address> addresses_;
};

std::ostream& operator <<(
        std::ostream& output,
        const DiscoveryServerConnectionAddress& address);

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TYPES_ADDRESS_DISCOVERYSERVERCONNECTIONADDRESS_HPP_ */
