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
 * @file AddressConfiguration.hpp
 *
 */

#ifndef _DDSROUTER_TYPES_ADDRESS_ADDRESSCONFIGURATION_HPP_
#define _DDSROUTER_TYPES_ADDRESS_ADDRESSCONFIGURATION_HPP_

#include <string>
#include <vector>

#include <ddsrouter/configuration/ParticipantConfiguration.hpp>
#include <ddsrouter/types/address/Address.hpp>
#include <ddsrouter/types/dds_types.hpp>
#include <ddsrouter/types/address/DiscoveryServerAddress.hpp>
#include <ddsrouter/types/endpoint/Guid.hpp>
#include <ddsrouter/types/RawConfiguration.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

DomainId domain(const RawConfiguration& configuration);

std::vector<Address> listening_addresses(const RawConfiguration& configuration);

std::vector<DiscoveryServerAddress> connection_addresses(const RawConfiguration& configuration);

Guid discovery_server_guid(const RawConfiguration& configuration);

Address get_address_from_configuration_(
    const RawConfiguration& configuration);

DiscoveryServerAddress get_discovery_server_address_from_configuration_(
    const RawConfiguration& configuration);

constexpr const DomainId DEFAULT_DOMAIN_ID(0);

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TYPES_ADDRESS_ADDRESSCONFIGURATION_HPP_ */
