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

namespace eprosima {
namespace ddsrouter {

using IpType = std::string;
using PortType = uint16_t;
using DiscoveryServerIdType = uint32_t;

enum IpVersion
{
    IPv4,
    IPv6
};

class Address
{
    Address(
        const IpType& ip,
        const PortType& port);

    PortType port() const noexcept;
    IpType ip() const noexcept;
    IpVersion ip_version() const noexcept;

    static bool is_ipv4_correct(
            const IpType& ip);

    static bool is_ipv6_correct(
            const IpType& ip);

protected:

    IpType ip_;
    PortType port_;
    IpVersion ip_version_;
};

std::ostream& operator <<(
        std::ostream& output,
        const Address& address);

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TYPES_ADDRESS_ADDRESS_HPP_ */
