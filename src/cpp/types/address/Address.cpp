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

#include <fastrtps/utils/IPLocator.h>

#include <ddsrouter/exceptions/DNSException.hpp>
#include <ddsrouter/types/address/Address.hpp>
#include <ddsrouter/types/utils.hpp>

namespace eprosima {
namespace ddsrouter {

const PortType Address::DEFAULT_PORT_ = 11666;
const IpType Address::DEFAULT_IP_v4_ = "127.0.0.1";
const IpType Address::DEFAULT_IP_v6_ = "::1";
const IpVersion Address::DEFAULT_IP_VERSION_ = IPv4;
const TransportProtocol Address::DEFAULT_TRANSPORT_PROTOCOL_ = UDP;

Address::Address()
    : Address(DEFAULT_IP_v4_, DEFAULT_PORT_, DEFAULT_IP_VERSION_, DEFAULT_TRANSPORT_PROTOCOL_)
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
    , domain_()
    , has_domain_(false)
    , has_valid_domain_(false)
{
}

Address::Address(
        const PortType& port,
        const IpVersion& ip_version,
        const DomainType& domain,
        const TransportProtocol& transport_protocol) noexcept
    : ip_()
    , port_(port)
    , ip_version_(ip_version)
    , transport_protocol_(transport_protocol)
    , domain_(domain)
    , has_domain_(true)
    , has_valid_domain_(false)
{
    try
    {
        ip_ = Address::resolve_dns(domain_, ip_version_);
        has_valid_domain_ = true;
    }
    catch (const DNSException& e)
    {
        logWarning(
            DDSROUTER_ADDRESS, "Address created without IP because given domain " << domain << " was not found.");
    }
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

Address::Address(
        const PortType& port,
        const DomainType& domain,
        const TransportProtocol& transport_protocol) noexcept
    : ip_()
    , port_(port)
    , transport_protocol_(transport_protocol)
    , domain_(domain)
    , has_domain_(true)
    , has_valid_domain_(false)
{
    try
    {
        auto dns_respone = Address::resolve_dns(domain_);
        ip_ = dns_respone.first;
        ip_version_ = dns_respone.second;
        has_valid_domain_ = true;
    }
    catch (const DNSException& e)
    {
        logWarning(
            DDSROUTER_ADDRESS, "Address created without IP because given domain " << domain << " was not found.");
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
    if (has_domain_ && !has_valid_domain_)
    {
        return false;
    }

    // TODO check port and maybe UDP/TCP specific rules
    switch (ip_version_)
    {
        case IPv4:
            return is_ipv4_correct(ip_);

        case IPv6:
            return is_ipv6_correct(ip_);

        default:
            utils::tsnh(
                utils::Formatter() << "Ip version value out of IpVersion.");
            return false; // Unreachable code
    }
}

bool Address::operator <(
        const Address& other) const noexcept
{
    // Compare IPs
    int ip_comparison = this->ip_.compare(other.ip_);
    if (ip_comparison < 0)
    {
        return true;
    }
    else if (ip_comparison > 0)
    {
        return false;
    }
    else
    {
        // Compare Ports
        if (this->port_ < other.port_)
        {
            return true;
        }
        else if (this->port_ > other.port_)
        {
            return false;
        }
        else
        {
            // Compare Protocol
            if (this->transport_protocol_ < other.transport_protocol_)
            {
                return true;
            }
            else if (this->transport_protocol_ > other.transport_protocol_)
            {
                return false;
            }
            else
            {
                // Compare Ip Version
                return this->ip_version_ < other.ip_version_;
            }
        }
    }
}

bool Address::operator ==(
        const Address& other) const noexcept
{
    return this->ip() == other.ip() &&
           this->port() == other.port() &&
           this->transport_protocol() == other.transport_protocol() &&
           this->is_valid() == other.is_valid();
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

IpType Address::default_ip(
        IpVersion ip_version /* = default_ip_version() */) noexcept
{
    if (ip_version == IPv4)
    {
        return DEFAULT_IP_v4_;
    }
    else if (ip_version == IPv6)
    {
        return DEFAULT_IP_v6_;
    }
    else
    {
        utils::tsnh(utils::Formatter() << "Value " << ip_version << " is not allowed.");
        return DEFAULT_IP_v4_;
    }
}

IpVersion Address::default_ip_version() noexcept
{
    return DEFAULT_IP_VERSION_;
}

TransportProtocol Address::default_transport_protocol() noexcept
{
    return DEFAULT_TRANSPORT_PROTOCOL_;
}

IpType Address::resolve_dns(
        DomainType domain,
        IpVersion ip_version)
{
    std::pair<std::set<std::string>, std::set<std::string>> dns_response =
            fastrtps::rtps::IPLocator::resolveNameDNS(domain);

    if (ip_version == IPv4)
    {
        if (dns_response.first.empty())
        {
            throw DNSException(
                      utils::Formatter() << "Could not resolve IPv4 for domain name <" << domain << ">.");
        }
        else
        {
            logInfo(
                DDSROUTER_ADDRESS,
                "Getting first IPv4: " << dns_response.first.begin()->data() <<
                    " for Domain name: " << domain <<
                    " from DNS response from " << dns_response.first.size() << " valid IPs.");
            return dns_response.first.begin()->data();
        }
    }
    else
    {
        if (dns_response.second.empty())
        {
            throw DNSException(
                      utils::Formatter() << "Could not resolve IPv6 for domain name <" << domain << ">.");
        }
        else
        {
            logInfo(
                DDSROUTER_ADDRESS,
                "Getting first IPv6: " << dns_response.second.begin()->data() <<
                    " for Domain name: " << domain <<
                    " from DNS response from " << dns_response.second.size() << " valid IPs.");
            return dns_response.second.begin()->data();
        }
    }
}

std::pair<IpType, IpVersion> Address::resolve_dns(
        DomainType domain)
{
    std::pair<std::set<std::string>, std::set<std::string>> dns_response =
            fastrtps::rtps::IPLocator::resolveNameDNS(domain);

    if (dns_response.first.empty())
    {
        if (dns_response.second.empty())
        {
            throw DNSException(
                      utils::Formatter() <<
                          "Could not resolve IP for IPv4 nor IPv6 for domain name <" << domain << ">.");
        }
        else
        {
            logInfo(
                DDSROUTER_ADDRESS,
                "Getting first IPv6: " << dns_response.second.begin()->data() <<
                    " for Domain name: " << domain <<
                    " from DNS response from " << dns_response.second.size() << " valid IPs.");
            return {dns_response.second.begin()->data(), IPv6};
        }
    }
    else
    {
        logInfo(
            DDSROUTER_ADDRESS,
            "Getting first IPv4: " << dns_response.first.begin()->data() <<
                " for Domain name: " << domain <<
                " from DNS response from " << (dns_response.first.size() + dns_response.second.size()) <<
                " valid IPs.");
        return {dns_response.first.begin()->data(), IPv4};
    }
}

std::ostream& operator <<(
        std::ostream& output,
        const Address& address)
{
    output << "{";

    if (address.has_domain_)
    {
        output << address.domain_ << "(" << address.ip() << ");";
    }
    else
    {
        output << address.ip() << ";";
    }
    output << address.port() << ";";

    if (address.is_udp())
    {
        output << "udp}";
    }
    else
    {
        // TODO change for else if cause there may be more transports
        output << "tcp}";
    }

    return output;
}

} /* namespace ddsrouter */
} /* namespace eprosima */
