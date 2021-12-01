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

#include <assert.h>

#include <fastrtps/utils/IPLocator.h>

#include <ddsrouter/types/address/Address.hpp>

namespace eprosima {
namespace ddsrouter {

const PortType Address::DEFAULT_PORT_ = 11666;
const IpType Address::DEFAULT_IP_ = "127.0.0.1";
const IpVersion Address::DEFAULT_IP_VERSION_ = IPv4;
const TransportProtocol Address::DEFAULT_TRANSPORT_PROTOCOL_ = UDP;

Address::Address()
    : Address(DEFAULT_IP_, DEFAULT_PORT_, DEFAULT_IP_VERSION_, DEFAULT_TRANSPORT_PROTOCOL_)
{
}

Address::Address(
        const IpType& ip,
        const PortType& port,
        const IpVersion& ip_version,
        const TransportProtocol& transport_protocol) noexcept
    : ip_(ip)
    , port_(port)
    , ip_version_(ip_version)
    , transport_protocol_(transport_protocol)
{
}

Address::Address(
        const IpType& ip,
        const PortType& port,
        const TransportProtocol& transport_protocol) noexcept
    : Address(ip, port, IPv4, transport_protocol)
{
    if (is_ipv6_correct(ip_))
    {
        ip_version_ = IPv6;
    }
}

PortType Address::port() const noexcept
{
    return port_;
}

IpType Address::ip() const noexcept
{
    return ip_;
}

IpVersion Address::ip_version() const noexcept
{
    return ip_version_;
}

TransportProtocol Address::transport_protocol() const noexcept
{
    return transport_protocol_;
}

bool Address::is_udp() const noexcept
{
    return transport_protocol_ == UDP;
}

bool Address::is_tcp() const noexcept
{
    return transport_protocol_ == TCP;
}

bool Address::is_ipv4() const noexcept
{
    return ip_version_ == IPv4;
}

bool Address::is_ipv6() const noexcept
{
    return ip_version_ == IPv6;
}

LocatorType Address::get_locator_kind() noexcept
{
    if (ip_version_ == IPv4)
    {
        if (transport_protocol_ == UDP)
        {
            return LOCATOR_KIND_UDPv4;
        }
        else if (transport_protocol_ == TCP)
        {
            return LOCATOR_KIND_TCPv4;
        }
    }
    else if (ip_version_ == IPv6)
    {
        if (transport_protocol_ == UDP)
        {
            return LOCATOR_KIND_UDPv6;
        }
        else if (transport_protocol_ == TCP)
        {
            return LOCATOR_KIND_TCPv6;
        }
    }

    return LOCATOR_KIND_INVALID;
}

bool Address::is_valid() const noexcept
{
    // TODO check port and maybe UDP/TCP specific rules
    switch (ip_version_)
    {
        case IPv4:
            return is_ipv4_correct(ip_);

        case IPv6:
            return is_ipv6_correct(ip_);

        default:
            assert(false);
            return false; // Unreachable code
    }
}

bool Address::is_ipv4_correct(
        const IpType& ip) noexcept
{
    return eprosima::fastrtps::rtps::IPLocator::isIPv4(ip);
}

bool Address::is_ipv6_correct(
        const IpType& ip) noexcept
{
    return eprosima::fastrtps::rtps::IPLocator::isIPv6(ip);
}

PortType Address::default_port() noexcept
{
    return DEFAULT_PORT_;
}

IpType Address::default_ip() noexcept
{
    return DEFAULT_IP_;
}

IpVersion Address::default_ip_version() noexcept
{
    return DEFAULT_IP_VERSION_;
}

TransportProtocol Address::default_transport_protocol() noexcept
{
    return DEFAULT_TRANSPORT_PROTOCOL_;
}

std::ostream& operator <<(
        std::ostream& output,
        const Address& address)
{
    output << "{" << address.ip() << ";" << address.port() << ";";

    if (address.is_udp())
    {
        output << "udp}";
    }
    else
    {
        // TODO change for else if cause it may be more transports
        output << "tcp}";
    }

    return output;
}

} /* namespace ddsrouter */
} /* namespace eprosima */
