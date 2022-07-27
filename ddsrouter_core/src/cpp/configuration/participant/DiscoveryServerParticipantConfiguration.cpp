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

DiscoveryServerParticipantConfiguration::DiscoveryServerParticipantConfiguration(
        const types::ParticipantId& id,
        const types::ParticipantKind& kind,
        const types::DomainId& domain_id,
        const types::GuidPrefix& discovery_server_guid_prefix,
        const std::set<types::Address>& listening_addresses,
        const std::set<types::DiscoveryServerConnectionAddress>& connection_addresses,
        const types::security::TlsConfiguration tls_configuration)
    : SimpleParticipantConfiguration(id, kind, domain_id)
    , discovery_server_guid_prefix_(discovery_server_guid_prefix)
    , listening_addresses_(listening_addresses)
    , connection_addresses_(connection_addresses)
    , tls_configuration_(tls_configuration)
{
    // Do nothing
}

bool DiscoveryServerParticipantConfiguration::is_valid(
        utils::Formatter& error_msg) const noexcept
{
    // Check parent class validity
    if (!SimpleParticipantConfiguration::is_valid(error_msg))
    {
        return false;
    }

    // Check DS Guid Prefix
    if (!discovery_server_guid_prefix_.is_valid())
    {
        error_msg << "Non valid Participant Guid Prefix " << discovery_server_guid_prefix_ << ". ";
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

    return true;
}

bool DiscoveryServerParticipantConfiguration::operator ==(
        const DiscoveryServerParticipantConfiguration& other) const noexcept
{
    // TODO
    return false;
}

} /* namespace configuration */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
