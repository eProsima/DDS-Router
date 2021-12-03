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

#include <ddsrouter/types/RawConfiguration.hpp>

namespace eprosima {
namespace ddsrouter {

// Data type for Locator kind (this is because FastDDS does not have an enum but defines (dishonor on your cow))
using LocatorType = uint32_t;
// Data Type for IP
using IpType = std::string;
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
    Address(
            const IpType& ip,
            const PortType& port,
            const IpVersion& ip_version,
            const TransportProtocol& transport_protocol) noexcept;

    /**
     * @brief Construct Address and get IP version from IP format
     *
     * If the IP is a string with format IPv4, version will be set to IPv4.
     * If the IP is a string with format IPv6, version will be set to IPv6.
     * If the IP has an incorrect formant, version will be set to IPv4 and address will be invalid.
     */
    Address(
            const IpType& ip,
            const PortType& port,
            const TransportProtocol& transport_protocol) noexcept;

    //! Construct a default IP by default values (set in this class)
    Address();

    //! Address Port getter
    PortType port() const noexcept;
    //! Address IP getter
    IpType ip() const noexcept;
    //! Address IP version getter
    IpVersion ip_version() const noexcept;
    //! Address transport protocol version getter
    TransportProtocol transport_protocol() const noexcept;

    //! Whether transport is UDP
    bool is_udp() const noexcept;
    //! Whether transport is TCP
    bool is_tcp() const noexcept;

    //! Whether ip version is IPv4
    bool is_ipv4() const noexcept;
    //! Whether ip version is IPv6
    bool is_ipv6() const noexcept;

    //! Get FastDDS Locator kind
    LocatorType get_locator_kind() noexcept;

    /**
     * @brief Whether the address is correct
     *
     * Checks if IP is in correct format regarding the IP version.
     * Checks if Port is correct.
     */
    virtual bool is_valid() const noexcept;

    //! Minor operator
    bool operator <(
            const Address& other) const noexcept;

    //! Whether string \c ip has correct IPv4 format.
    static bool is_ipv4_correct(
            const IpType& ip) noexcept;

    //! Whether string \c ip has correct IPv6 format.
    static bool is_ipv6_correct(
            const IpType& ip) noexcept;

    //! Default port for address when is not set: 11600
    static PortType default_port() noexcept;
    //! Default ip for address when is not set: 127.0.0.1
    static IpType default_ip() noexcept;
    //! Default ip version for address when is not set: IPv4
    static IpVersion default_ip_version() noexcept;
    //! Default transport protocol for address when is not set: UDP
    static TransportProtocol default_transport_protocol() noexcept;

    /////
    // YAML methods

    /**
     * @brief Construct a new Address from yaml object
     *
     * The tags required by addres are ADDRESS_|IP|PORT|IP_VERSION|TRANSPORT| .
     * This tags and their values will be in the base level of the yaml \c configuration .
     * This means the tags will be accessible as configuration[<tag>] and not inside further tags.
     *
     * @param configuration configuration where this address must be set
     * @param default_transport default transport protocol in case it is not set in yaml
     */
    Address(
        const RawConfiguration& configuration,
        TransportProtocol default_transport = Address::default_transport_protocol());

    //! Dump this object in \c configuration variable at \c configuration yaml base level.
    RawConfiguration dump(RawConfiguration& configuration) const; // TODO: Once implemented add noexcept

protected:

    //! Internal IP object
    IpType ip_;
    //! Internal Port object
    PortType port_;
    //! Internal Ip version object
    IpVersion ip_version_;
    //! Internal Transport Protocol object
    TransportProtocol transport_protocol_;

    //! Default Port
    static const PortType DEFAULT_PORT_;                         // 11666
    //! Default IP
    static const IpType DEFAULT_IP_;                             // 127.0.0.1
    //! Default IP version
    static const IpVersion DEFAULT_IP_VERSION_;                  // IPv4
    //! Default Transport protocol
    static const TransportProtocol DEFAULT_TRANSPORT_PROTOCOL_;  // UDP
};

//! \c Address to stream serializator
std::ostream& operator <<(
        std::ostream& output,
        const Address& address);

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TYPES_ADDRESS_ADDRESS_HPP_ */
