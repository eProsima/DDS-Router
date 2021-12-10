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
 * @file DiscoveryServerParticipantConfiguration.cpp
 */

#include <ddsrouter/configuration/DiscoveryServerParticipantConfiguration.hpp>
#include <ddsrouter/types/configuration_tags.hpp>
#include <ddsrouter/types/Log.hpp>
#include <ddsrouter/exceptions/ConfigurationException.hpp>

namespace eprosima {
namespace ddsrouter {

const TransportProtocol DiscoveryServerParticipantConfiguration::DEFAULT_TRANSPORT_PROTOCOL_ =
        TransportProtocol::UDP;

const DomainId DiscoveryServerParticipantConfiguration::DEFAULT_DS_DOMAIN_ID_(66u);

DiscoveryServerParticipantConfiguration::DiscoveryServerParticipantConfiguration(
        const ParticipantConfiguration& configuration)
    : ParticipantConfiguration(configuration.id(), configuration.raw_configuration())
{
}

std::vector<Address> DiscoveryServerParticipantConfiguration::listening_addresses() const
{
    std::vector<Address> result;

    if (raw_configuration_[LISTENING_ADDRESSES_TAG])
    {
        if (!raw_configuration_[LISTENING_ADDRESSES_TAG].IsSequence())
        {
            throw ConfigurationException("Listening addresses must be a sequence.");
        }

        for (auto address : raw_configuration_[LISTENING_ADDRESSES_TAG])
        {
            // In case some of it fails, throw exception forward
            result.push_back(Address(address, default_transport_protocol_()));
        }
    }

    return result;
}

std::vector<DiscoveryServerConnectionAddress> DiscoveryServerParticipantConfiguration::connection_addresses() const
{
    std::vector<DiscoveryServerConnectionAddress> result;

    if (raw_configuration_[CONNECTION_ADDRESSES_TAG])
    {
        if (!raw_configuration_[CONNECTION_ADDRESSES_TAG].IsSequence())
        {
            throw ConfigurationException("Listening addresses must be a sequence.");
        }

        for (auto address : raw_configuration_[CONNECTION_ADDRESSES_TAG])
        {
            // In case some of it fails, throw exception forward
            result.push_back(DiscoveryServerConnectionAddress(address, default_transport_protocol_()));
        }
    }

    return result;
}

GuidPrefix DiscoveryServerParticipantConfiguration::discovery_server_guid() const
{
    return GuidPrefix(raw_configuration_);
}

DomainId DiscoveryServerParticipantConfiguration::domain() const
{
    // This method always return the same domain for the Discovery Server in order
    // to avoid problems with the LogicalPort
    return DEFAULT_DS_DOMAIN_ID_;
}

TransportProtocol DiscoveryServerParticipantConfiguration::default_transport_protocol_() const noexcept
{
    return DEFAULT_TRANSPORT_PROTOCOL_;
}

} /* namespace ddsrouter */
} /* namespace eprosima */
