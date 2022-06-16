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
#include <ddsrouter_utils/exception/ConfigurationException.hpp>
#include <ddsrouter_utils/Log.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace configuration {

using namespace eprosima::ddsrouter::core::types;

DiscoveryServerParticipantConfiguration::DiscoveryServerParticipantConfiguration(
        const ParticipantId& id,
        const GuidPrefix& discovery_server_guid_prefix,
        const std::set<Address>& listening_addresses,
        const std::set<DiscoveryServerConnectionAddress>& connection_addresses,
        const DomainId domain_id /* = DEFAULT_DS_DOMAIN_ID */,
        const types::security::TlsConfiguration& tls_configuration /* = types::security::TlsConfiguration() */)
    : SimpleParticipantConfiguration(id, domain_id)
    , discovery_server_guid_prefix_(discovery_server_guid_prefix)
    , listening_addresses_(listening_addresses)
    , connection_addresses_(connection_addresses)
    , tls_configuration_(tls_configuration)
{
    // Check exist at least one address
    if (listening_addresses_.empty() && connection_addresses_.empty())
    {
        throw utils::ConfigurationException( utils::Formatter() << "No listening or connection address specified. ");
    }

    // If Tls configuration is defined
    if (tls_configuration_.is_active())
    {
        // If has listening addresses, it should be able to provide TLS server configuration
        if (!listening_addresses_.empty())
        {
            if (!tls_configuration_.compatible<types::security::TlsKind::server>())
            {
                throw utils::ConfigurationException(
                          utils::Formatter() <<
                              "TLS requires to support Server Configuration if listening addresses set. ");
            }
        }

        // If has connection addresses, it should be able to provide TLS client configuration
        if (!connection_addresses_.empty())
        {
            if (!tls_configuration_.compatible<types::security::TlsKind::client>())
            {
                throw utils::ConfigurationException(
                          utils::Formatter() <<
                              "TLS requires to support Client Configuration if connection addresses set. ");
            }
        }
    }

    // Check DS Guid Prefix
    if (!discovery_server_guid_prefix_.is_valid())
    {
        throw utils::ConfigurationException(
                  utils::Formatter() << "Non valid Participant Guid Prefix " << discovery_server_guid_prefix_ <<
                      ". ");
    }
}

const GuidPrefix& DiscoveryServerParticipantConfiguration::discovery_server_guid_prefix() const noexcept
{
    return discovery_server_guid_prefix_;
}

const std::set<Address>& DiscoveryServerParticipantConfiguration::listening_addresses() const noexcept
{
    return listening_addresses_;
}

const std::set<DiscoveryServerConnectionAddress>&
DiscoveryServerParticipantConfiguration::connection_addresses() const noexcept
{
    return connection_addresses_;
}

const types::security::TlsConfiguration& DiscoveryServerParticipantConfiguration::tls_configuration() const noexcept
{
    return tls_configuration_;
}

} /* namespace configuration */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
