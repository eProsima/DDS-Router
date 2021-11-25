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
 * @file Address.hpp
 *
 */

#ifndef _DDSROUTER_TYPES_ADDRESS_ADDRESS_HPP_
#define _DDSROUTER_TYPES_ADDRESS_ADDRESS_HPP_

#include <string>

#include <fastdds/rtps/common/Locator.h>

namespace eprosima {
namespace ddsrouter {

using LocatorType = uint32_t;
using IpType = std::string;
using PortType = uint16_t;
using DiscoveryServerIdType = uint32_t;

enum IpVersion
{
    IPv4,
    IPv6
};

enum TransportProtocol
{
    UDP,
    TCP
};

class Address
{
public:

    Address(
        const IpType& ip,
        const PortType& port,
        const IpVersion& ip_version,
        const TransportProtocol& transport_protocol) noexcept;

    Address(
        const IpType& ip,
        const PortType& port,
        const TransportProtocol& transport_protocol) noexcept;

    PortType port() const noexcept;
    IpType ip() const noexcept;
    IpVersion ip_version() const noexcept;
    TransportProtocol transport_protocol() const noexcept;

    bool is_udp() const noexcept;
    bool is_tcp() const noexcept;

    bool is_ipv4() const noexcept;
    bool is_ipv6() const noexcept;

    LocatorType get_locator_kind() noexcept;

    virtual bool is_valid() const noexcept;

    static bool is_ipv4_correct(
            const IpType& ip) noexcept;

    static bool is_ipv6_correct(
            const IpType& ip) noexcept;

    static PortType default_port() noexcept;
    static IpType default_ip() noexcept;
    static IpVersion default_ip_version() noexcept;
    static TransportProtocol default_transport_protocol() noexcept;

protected:

    IpType ip_;
    PortType port_;
    IpVersion ip_version_;
    TransportProtocol transport_protocol_;

    static const PortType DEFAULT_PORT_TYPE;
    static const IpType DEFAULT_IP_TYPE;
    static const IpVersion DEFAULT_IP_VERSION;
    static const TransportProtocol DEFAULT_TRANSPORT_PROTOCOL;
};

std::ostream& operator <<(
        std::ostream& output,
        const Address& address);

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TYPES_ADDRESS_ADDRESS_HPP_ */
