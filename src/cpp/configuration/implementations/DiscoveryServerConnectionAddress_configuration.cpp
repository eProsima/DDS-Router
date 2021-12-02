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
 * @file DiscoveryServerConnectionAddress_configuration.cpp
 *
 */

#include <ddsrouter/exceptions/ConfigurationException.hpp>
#include <ddsrouter/exceptions/UnsupportedException.hpp>
#include <ddsrouter/types/address/Address.hpp>
#include <ddsrouter/types/address/DiscoveryServerConnectionAddress.hpp>
#include <ddsrouter/types/configuration_tags.hpp>
#include <ddsrouter/types/RawConfiguration.hpp>

namespace eprosima {
namespace ddsrouter {

DiscoveryServerConnectionAddress::DiscoveryServerConnectionAddress(
        const RawConfiguration& configuration,
        TransportProtocol default_transport /*= Address::default_transport_protocol()*/)
{
    // Get DS guid. If it fails, throw exception forward
    discovery_server_guid_ = GuidPrefix(configuration);

    // Get Addresses from a list
    if (configuration[COLLECTION_ADDRESSES_TAG])
    {
        // It must be a sequence
        if (!configuration[COLLECTION_ADDRESSES_TAG].IsSequence())
        {
            throw ConfigurationException(
                utils::Formatter() <<
                "Discovery Server Connection Address must have sequence of address under tag: " <<
                COLLECTION_ADDRESSES_TAG);
        }

        // For each element in sequence, get address
        for (auto address : configuration[COLLECTION_ADDRESSES_TAG])
        {
            // In case some of it fails, throw exception forward
            addresses_.insert(Address(address, default_transport));
        }
    }
}

RawConfiguration DiscoveryServerConnectionAddress::dump(RawConfiguration&) const
{
    // TODO
    throw UnsupportedException("DiscoveryServerConnectionAddress::dump is not supported yet.");
}

} /* namespace ddsrouter */
} /* namespace eprosima */
