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

#ifndef _DDSROUTERCORE_TYPES_ADDRESS_ADDRESS_HPP_
#define _DDSROUTERCORE_TYPES_ADDRESS_ADDRESS_HPP_

#include <string>

#include <fastdds/rtps/common/Locator.h>

#include <ddsrouter_core/library/library_dll.h>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

// Data type for Locator kind (this is because FastDDS does not have an enum but defines (dishonor on your cow))
using LocatorType = uint32_t;
// Data Type for IP
using IpType = std::string;
// Data Type for Domain name (DNS)
using DomainType = std::string;
// Data Type for Port
using PortType = uint16_t;

//! Different versions allowed for IP
enum IpVersion
{
    IPv4,
    IPv6
};

//! Different Transport Protocols allowed
enum TransportProtocol
{
    UDP,
    TCP
};

/**
 * @brief Address that works as a collection of data defining a network address.
 *
 * An address will remain in an IP and a Port, that will uniquely define a network address for connection
 * or listening.
 * There are extra fields in the class such as IP version and Transport Protocol that are settable by the constructor.
 */
class Address
{
public:

    /**
     * @brief Construct a new Address object with all the parameters
     *
     * @param ip address ip
     * @param port address port
     * @param ip_version ip version (4 or 6)
     * @param transport_protocol transport protocol (UDP or TCP)
     */
    DDSROUTER_CORE_DllAPI Address(
            const IpType& ip,
            const PortType& port,
            const IpVersion& ip_version,
            const TransportProtocol& transport_protocol) noexcept;

    /**
     * @brief Construct a new Address object using a DNS call to get IP from \c domain
     *
     * @param port address port
     * @param ip_version ip version (4 or 6)
     * @param domain address domain name to call DNS
     * @param transport_protocol transport protocol (UDP or TCP)
     */
    DDSROUTER_CORE_DllAPI Address(
            const PortType& port,
            const IpVersion& ip_version,
            const DomainType& domain,
            const TransportProtocol& transport_protocol) noexcept;

    /**
     * @brief Construct Address and get IP version from IP format
     *
     * If the IP is a string with format IPv4, version will be set to IPv4.
     * If the IP is a string with format IPv6, version will be set to IPv6.
     * If the IP has an incorrect formant, version will be set to IPv4 and address will be invalid.
     */
    DDSROUTER_CORE_DllAPI Address(
            const IpType& ip,
            const PortType& port,
            const TransportProtocol& transport_protocol) noexcept;

    /**
     * @brief Construct a new Address object using a DNS call to get IP from \c domain without specifying the IP version
     *
     * @param port address port
     * @param domain address domain name to call DNS
     * @param transport_protocol transport protocol (UDP or TCP)
     */
    DDSROUTER_CORE_DllAPI Address(
            const PortType& port,
            const DomainType& domain,
            const TransportProtocol& transport_protocol) noexcept;

    //! Construct a default IP by default values (set in this class)
    DDSROUTER_CORE_DllAPI Address();

    //! Address Port getter
    DDSROUTER_CORE_DllAPI PortType port() const noexcept;
    //! Address IP getter
    DDSROUTER_CORE_DllAPI IpType ip() const noexcept;
    //! Address IP version getter
    DDSROUTER_CORE_DllAPI IpVersion ip_version() const noexcept;
    //! Address transport protocol version getter
    DDSROUTER_CORE_DllAPI TransportProtocol transport_protocol() const noexcept;

    //! Whether transport is UDP
    DDSROUTER_CORE_DllAPI bool is_udp() const noexcept;
    //! Whether transport is TCP
    DDSROUTER_CORE_DllAPI bool is_tcp() const noexcept;

    //! Whether ip version is IPv4
    DDSROUTER_CORE_DllAPI bool is_ipv4() const noexcept;
    //! Whether ip version is IPv6
    DDSROUTER_CORE_DllAPI bool is_ipv6() const noexcept;

    //! Get FastDDS Locator kind
    DDSROUTER_CORE_DllAPI LocatorType get_locator_kind() noexcept;

    /**
     * @brief Whether the address is correct
     *
     * Checks if IP is in correct format regarding the IP version.
     * Checks if Port is correct.
     */
    DDSROUTER_CORE_DllAPI virtual bool is_valid() const noexcept;

    //! Minor operator
    DDSROUTER_CORE_DllAPI bool operator <(
            const Address& other) const noexcept;

    //! Equal operator
    DDSROUTER_CORE_DllAPI bool operator ==(
            const Address& other) const noexcept;

    //! Whether string \c ip has correct IPv4 format.
    DDSROUTER_CORE_DllAPI static bool is_ipv4_correct(
            const IpType& ip) noexcept;

    //! Whether string \c ip has correct IPv6 format.
    DDSROUTER_CORE_DllAPI static bool is_ipv6_correct(
            const IpType& ip) noexcept;

    //! Default port for address when is not set: 11600
    DDSROUTER_CORE_DllAPI static PortType default_port() noexcept;
    //! Default ip for address when is not set: 127.0.0.1
    DDSROUTER_CORE_DllAPI static IpType default_ip(
            IpVersion ip_version = default_ip_version()) noexcept;
    //! Default ip version for address when is not set: IPv4
    DDSROUTER_CORE_DllAPI static IpVersion default_ip_version() noexcept;
    //! Default transport protocol for address when is not set: UDP
    DDSROUTER_CORE_DllAPI static TransportProtocol default_transport_protocol() noexcept;

    /**
     * @brief Return the IP corresponding to the \c domain name given with IP version specified in \c ip_version
     *
     * Make a DNS call to get the IP related with \c domain
     *
     * @param domain domain name of the ip to look for
     * @param ip_version version of the ip to find
     *
     * @return IpType IP related with \c domain name
     *
     * @throw DNSException in case an IP for this domain could not be retrieved
     */
    DDSROUTER_CORE_DllAPI static IpType resolve_dns(
            DomainType domain,
            IpVersion ip_version);

    /**
     * @brief Return the IP corresponding to the \c domain name given
     *
     * Make a DNS call to get the IP related with \c domain .
     * Get the IP found in default IP Version (IPv4) if present. If not, find IPv6.
     *
     * @param domain domain name of the ip to look for
     *
     * @return IpType IP related with \c domain name.
     * @return IpVersion version of the IP found.
     *
     * @throw DNSException in case an IP for this domain could not be retrieved
     */
    DDSROUTER_CORE_DllAPI static std::pair<IpType, IpVersion> resolve_dns(
            DomainType domain);

protected:

    //! Internal IP object
    IpType ip_;
    //! Domain Name with which this Address has been created (in case it is created with ip, it is not used)
    DomainType domain_;
    //! Whether this Address has been initialized with domain or not
    bool has_domain_;
    //! Whether the domain has been valid on DNS call
    bool has_valid_domain_;
    //! Internal Port object
    PortType port_;
    //! Internal Ip version object
    IpVersion ip_version_;
    //! Internal Transport Protocol object
    TransportProtocol transport_protocol_;

    //! Default Port
    static const PortType DEFAULT_PORT_;                         // 11666
    //! Default IPv4
    static const IpType DEFAULT_IP_v4_;                             // 127.0.0.1
    //! Default IPv6
    static const IpType DEFAULT_IP_v6_;                             // ::1
    //! Default IP version
    static const IpVersion DEFAULT_IP_VERSION_;                  // IPv4
    //! Default Transport protocol
    static const TransportProtocol DEFAULT_TRANSPORT_PROTOCOL_;  // UDP

    // Allow operator << to use domain info
    DDSROUTER_CORE_DllAPI friend std::ostream& operator <<(
            std::ostream& output,
            const Address& address);
};

//! \c Address to stream serializator
DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& output,
        const Address& address);

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_ADDRESS_ADDRESS_HPP_ */
