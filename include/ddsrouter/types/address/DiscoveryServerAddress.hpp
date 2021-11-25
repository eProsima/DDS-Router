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
 * @file DiscoveryServerAddress.hpp
 *
 */

#ifndef _DDSROUTER_TYPES_ADDRESS_DISCOVERYSERVERADDRESS_HPP_
#define _DDSROUTER_TYPES_ADDRESS_DISCOVERYSERVERADDRESS_HPP_

#include <string>
#include <vector>

#include <ddsrouter/types/address/Address.hpp>
#include <ddsrouter/types/endpoint/Guid.hpp>

namespace eprosima {
namespace ddsrouter {

class DiscoveryServerAddress : public Address
{
public:

    DiscoveryServerAddress(
        const IpType& ip,
        const PortType& port,
        const IpVersion& ip_version,
        const TransportProtocol& transport_protocol,
        const GuidPrefix& server_guid);

    DiscoveryServerAddress(
        const IpType& ip,
        const PortType& port,
        const IpVersion& ip_version,
        const TransportProtocol& transport_protocol,
        const uint32_t server_guid_seed);

    DiscoveryServerAddress(
        const IpType& ip,
        const PortType& port,
        const TransportProtocol& transport_protocol,
        const GuidPrefix& server_guid);

    DiscoveryServerAddress(
        const IpType& ip,
        const PortType& port,
        const TransportProtocol& transport_protocol,
        const uint32_t server_guid_seed);

    GuidPrefix discovery_server_guid() const noexcept;

    virtual bool is_valid() const noexcept;

protected:

    GuidPrefix discovery_server_guid_;
};

std::ostream& operator <<(
        std::ostream& output,
        const DiscoveryServerAddress& address);

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TYPES_ADDRESS_DISCOVERYSERVERADDRESS_HPP_ */
