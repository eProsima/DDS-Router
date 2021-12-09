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

/**
 * @brief Collection of Addresses to connect with a remote Disovery Server.
 *
 * An address will remain in an IP and a Port, IP version and Transport Protocol.
 * This class has several address associated with one \c GuidPrefix in order to connect with
 * a remote Discovery Server.
 */class DiscoveryServerConnectionAddress
{
public:

    /**
     * @brief Construct a new \c DiscoveryServerConnectionAddress object with all the parameters
     *
     * @param discovery_server_guid_ : Guid Prefix of the remote Discovery Server
     * @param addresses_ collection of addresses
     */
    DiscoveryServerConnectionAddress(
            GuidPrefix discovery_server_guid,
            std::set<Address> addresses);

    //! Discovery Server \c GuidPrefix Port getter
    GuidPrefix discovery_server_guid() const noexcept;

    //! Addresses getter
    std::set<Address> addresses() const noexcept;

    /**
     * @brief Whether the address is correct
     *
     * Checks if GuidPrefix is correct.
     * Checks if it has at least one correct address.
     */
    virtual bool is_valid() const noexcept;

    /////
    // YAML methods

    /**
     * @brief Construct a new DiscoveryServer connection address from yaml object
     *
     * @param configuration : configuration where this address must be set (tags will be looked up in this same level)
     * @param default_transport : default transport protocol in case it is not set in yaml
     */
    DiscoveryServerConnectionAddress(
            const RawConfiguration& configuration,
            TransportProtocol default_transport = Address::default_transport_protocol());

    //! Dump this object in \c configuration variable at \c configuration yaml base level.
    RawConfiguration dump(
            RawConfiguration& configuration) const;               // TODO: Once implemented add noexcept

protected:

    //! Internal Discovery Server Guid Prefix object
    GuidPrefix discovery_server_guid_;

    //! Internal Addresses object
    std::set<Address> addresses_;
};

//! \c DiscoveryServerConnectionAddress to stream serializator
std::ostream& operator <<(
        std::ostream& output,
        const DiscoveryServerConnectionAddress& address);

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TYPES_ADDRESS_DISCOVERYSERVERCONNECTIONADDRESS_HPP_ */
