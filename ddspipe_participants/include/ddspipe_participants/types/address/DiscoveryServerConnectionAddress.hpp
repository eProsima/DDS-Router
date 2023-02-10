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

#pragma once

#include <string>
#include <vector>

#include <ddspipe_participants/library/library_dll.h>
#include <ddspipe_participants/types/address/Address.hpp>
#include <ddspipe_core/types/dds/Guid.hpp>

namespace eprosima {
namespace ddspipe {
namespace participants {
namespace types {

/**
 * @brief Collection of Addresses to connect with a remote Disovery Server.
 *
 * An address will remain in an IP and a Port, IP version and Transport Protocol.
 * This class has several address associated with one \c GuidPrefix in order to connect with
 * a remote Discovery Server.
 */
class DiscoveryServerConnectionAddress
{
public:

    /**
     * @brief Construct a new \c DiscoveryServerConnectionAddress object with all the parameters
     *
     * @param discovery_server_guid_ : Guid Prefix of the remote Discovery Server
     * @param addresses_ collection of addresses
     */
    DDSPIPE_PARTICIPANTS_DllAPI DiscoveryServerConnectionAddress(
            core::types::GuidPrefix discovery_server_guid,
            std::set<Address> addresses);

    //! Discovery Server \c GuidPrefix Port getter
    DDSPIPE_PARTICIPANTS_DllAPI core::types::GuidPrefix discovery_server_guid_prefix() const noexcept;

    //! Addresses getter
    DDSPIPE_PARTICIPANTS_DllAPI std::set<Address> addresses() const noexcept;

    /**
     * @brief Whether the address is correct
     *
     * Checks if GuidPrefix is correct.
     * Checks if it has at least one correct address.
     */
    DDSPIPE_PARTICIPANTS_DllAPI virtual bool is_valid() const noexcept;

    //! Minor operator
    DDSPIPE_PARTICIPANTS_DllAPI bool operator <(
            const DiscoveryServerConnectionAddress& other) const noexcept;

    //! Equal operator
    DDSPIPE_PARTICIPANTS_DllAPI bool operator ==(
            const DiscoveryServerConnectionAddress& other) const noexcept;

protected:

    //! Internal Discovery Server Guid Prefix object
    core::types::GuidPrefix discovery_server_guid_prefix_;

    //! Internal Addresses object
    std::set<Address> addresses_;
};

//! \c DiscoveryServerConnectionAddress to stream serializator
DDSPIPE_PARTICIPANTS_DllAPI std::ostream& operator <<(
        std::ostream& output,
        const DiscoveryServerConnectionAddress& address);

} /* namespace types */
} /* namespace participants */
} /* namespace ddspipe */
} /* namespace eprosima */
