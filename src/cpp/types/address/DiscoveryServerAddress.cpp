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
 * @file Address.cpp
 *
 */

#include <ddsrouter/types/address/DiscoveryServerAddress.hpp>

namespace eprosima {
namespace ddsrouter {

DiscoveryServerAddress::DiscoveryServerAddress(
        const IpType& ip,
        const PortType& port,
        const TransportProtocol& transport_protocol,
        const GuidPrefix& server_guid)
    : Address(ip, port, transport_protocol)
    , discovery_server_guid_(server_guid)
{
}

DiscoveryServerAddress::DiscoveryServerAddress(
        const IpType& ip,
        const PortType& port,
        const TransportProtocol& transport_protocol,
        const uint32_t server_guid_seed)
    : Address(ip, port, transport_protocol)
    , discovery_server_guid_(Guid::get_guid_prefix_from_seed(server_guid_seed))
{
}

bool DiscoveryServerAddress::is_valid() const noexcept
{
    // This ugly implementation is because c++ is so silly it does not support parallel inheritance
    return discovery_server_guid_ != GuidPrefix::unknown() && Address::is_valid();
}

GuidPrefix DiscoveryServerAddress::discovery_server_guid() const noexcept
{
    return discovery_server_guid_;
}

std::ostream& operator <<(
        std::ostream& output,
        const DiscoveryServerAddress& address)
{
    output << "{" << address.ip() << ";" << address.port() << ";";

    if (address.is_ipv4())
    {
        output << "udp;";
    }
    else
    {
        output << "tcp;";
    }

    output << address.discovery_server_guid();

    output << "}";

    return output;
}

} /* namespace ddsrouter */
} /* namespace eprosima */
