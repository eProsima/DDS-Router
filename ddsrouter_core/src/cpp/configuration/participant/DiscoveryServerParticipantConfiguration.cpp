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

#include <ddsrouter_core/configuration/participant/DiscoveryServerParticipantConfiguration.hpp>
#include <ddsrouter_utils/exception/ConfigurationException.hpp>
#include <ddsrouter_core/types/security/tls/TlsConfiguration.hpp>
#include <ddsrouter_utils/Log.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace configuration {

using namespace eprosima::ddsrouter::core::types;

const DomainId DiscoveryServerParticipantConfiguration::DEFAULT_DS_DOMAIN_ID_(66u);

DiscoveryServerParticipantConfiguration::DiscoveryServerParticipantConfiguration(
        const ParticipantId& id,
        const GuidPrefix& discovery_server_guid_prefix,
        const std::set<Address>& listening_addresses,
        const std::set<DiscoveryServerConnectionAddress>& connection_addresses,
        const ParticipantKind& kind /* = ParticipantKind::local_discovery_server */,
        const types::security::TlsConfiguration tls_configuration /* = types::security::TlsConfiguration() */,
        const DomainId& domain_id /* = DEFAULT_DS_DOMAIN_ID_ */)
    : SimpleParticipantConfiguration(id, kind, domain_id)
    , discovery_server_guid_prefix_(discovery_server_guid_prefix)
    , listening_addresses_(listening_addresses)
    , connection_addresses_(connection_addresses)
    , tls_configuration_(tls_configuration)
{
}

DiscoveryServerParticipantConfiguration::DiscoveryServerParticipantConfiguration(
        const ParticipantId& id,
        const GuidPrefix& discovery_server_guid_prefix,
        const std::set<Address>& listening_addresses,
        const std::set<DiscoveryServerConnectionAddress>& connection_addresses,
        const DomainId& domain_id,
        const ParticipantKind& kind /* = ParticipantKind::local_discovery_server */,
        const types::security::TlsConfiguration tls_configuration /* = types::security::TlsConfiguration() */)
    : DiscoveryServerParticipantConfiguration(
        id, discovery_server_guid_prefix, listening_addresses, connection_addresses, kind, tls_configuration, domain_id)
{
}

GuidPrefix DiscoveryServerParticipantConfiguration::discovery_server_guid_prefix() const noexcept
{
    return discovery_server_guid_prefix_;
}

std::set<Address> DiscoveryServerParticipantConfiguration::listening_addresses() const noexcept
{
    return listening_addresses_;
}

std::set<DiscoveryServerConnectionAddress>
DiscoveryServerParticipantConfiguration::connection_addresses() const noexcept
{
    return connection_addresses_;
}

bool DiscoveryServerParticipantConfiguration::tls_active() const noexcept
{
    return tls_configuration_.is_active();
}

const types::security::TlsConfiguration& DiscoveryServerParticipantConfiguration::tls_configuration() const noexcept
{
    return tls_configuration_;
}

bool DiscoveryServerParticipantConfiguration::is_valid(
        utils::Formatter& error_msg) const noexcept
{
    // Check parent class validity
    if (!SimpleParticipantConfiguration::is_valid(error_msg))
    {
        return false;
    }

    // Check listening addresses
    for (Address address : listening_addresses_)
    {
        if (!address.is_valid())
        {
            error_msg << "Incorrect address " << address << " in listening addresses. ";
            return false;
        }
    }

    // Check connection addresses
    for (DiscoveryServerConnectionAddress address : connection_addresses_)
    {
        if (!address.is_valid())
        {
            error_msg << "Incorrect address " << address << " in connection addresses. ";
            return false;
        }
    }

    // Check exist at least one address
    if (listening_addresses_.empty() && connection_addresses_.empty())
    {
        error_msg << "No listening or connection address specified. ";
        return false;
    }

    // If active, check it is valid
    if (tls_configuration_.is_active())
    {
        // If has listening addresses, it should be able to provide TLS server configuration
        if (!listening_addresses_.empty())
        {
            if (!tls_configuration_.compatible<types::security::TlsKind::server>())
            {
                error_msg << "TLS requires to support Server Configuration if listening addresses set. ";
                return false;
            }
        }

        // If has connection addresses, it should be able to provide TLS client configuration
        if (!connection_addresses_.empty())
        {
            if (!tls_configuration_.compatible<types::security::TlsKind::client>())
            {
                error_msg << "TLS requires to support Client Configuration if connection addresses set. ";
                return false;
            }
        }
    }

    // Check DS Guid Prefix
    if (!discovery_server_guid_prefix_.is_valid())
    {
        error_msg << "Non valid Participant Guid Prefix " << discovery_server_guid_prefix_ << ". ";
        return false;
    }

    return true;
}

bool DiscoveryServerParticipantConfiguration::operator ==(
        const DiscoveryServerParticipantConfiguration& other) const noexcept
{
    // TODO
    return false;
}

DomainId DiscoveryServerParticipantConfiguration::default_domain_id() noexcept
{
    return DEFAULT_DS_DOMAIN_ID_;
}

} /* namespace configuration */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
