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

#include <ddsrouter/configuration/participant/DiscoveryServerParticipantConfiguration.hpp>
#include <ddsrouter/types/configuration_tags.hpp>
#include <ddsrouter/types/Log.hpp>
#include <ddsrouter/exceptions/ConfigurationException.hpp>

namespace eprosima {
namespace ddsrouter {
namespace configuration {

const DomainId DiscoveryServerParticipantConfiguration::DEFAULT_DS_DOMAIN_ID_(66u);

DiscoveryServerParticipantConfiguration::DiscoveryServerParticipantConfiguration(
        const ParticipantId& id,
        const GuidPrefix& discovery_server_guid_prefix,
        const std::set<std::shared_ptr<Address>>& listening_addresses,
        const std::set<std::shared_ptr<DiscoveryServerConnectionAddress>>& connection_addresses,
        const std::map<std::string, std::string>& tls_configuration /* = {} */,
        const ParticipantType& type /* = ParticipantType::LOCAL_DISCOVERY_SERVER */,
        const DomainId& domain_id /* = DEFAULT_DS_DOMAIN_ID_ */)
    : SimpleParticipantConfiguration(id, type, domain_id)
    , discovery_server_guid_(discovery_server_guid_prefix)
    , listening_addresses_(listening_addresses)
    , connection_addresses_(connection_addresses)
    , tls_configuration_(tls_configuration)
{
}

GuidPrefix DiscoveryServerParticipantConfiguration::discovery_server_guid_prefix() const noexcept
{
    return discovery_server_guid_;
}

std::set<std::shared_ptr<Address>> DiscoveryServerParticipantConfiguration::listening_addresses() const noexcept
{
    return listening_addresses_;
}

std::set<std::shared_ptr<DiscoveryServerConnectionAddress>>
    DiscoveryServerParticipantConfiguration::connection_addresses() const noexcept
{
    return connection_addresses_;
}

bool DiscoveryServerParticipantConfiguration::tls_active() const noexcept
{
    return !tls_configuration_.empty();
}

std::map<std::string, std::string> DiscoveryServerParticipantConfiguration::tls_configuration() const
{
    return tls_configuration_;
}

bool DiscoveryServerParticipantConfiguration::is_valid() const noexcept
{
    // Check listening addresses
    for (std::shared_ptr<Address> address : listening_addresses_)
    {
        if (address->is_valid())
        {
            return false;
        }
    }

    // Check connection addresses
    for (std::shared_ptr<DiscoveryServerConnectionAddress> address : connection_addresses_)
    {
        if (address->is_valid())
        {
            return false;
        }
    }

    // TODO
    // More logic could be added, for example connection or listening is required

    // TODO
    // Check TLS

    return SimpleParticipantConfiguration::is_valid() &&
        discovery_server_guid_.is_valid();
}

bool DiscoveryServerParticipantConfiguration::operator ==(
        const DiscoveryServerParticipantConfiguration& other) const noexcept
{
    // TODO
    return false;
}

} /* namespace configuration */
} /* namespace ddsrouter */
} /* namespace eprosima */
