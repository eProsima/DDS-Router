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
 * @file DiscoveryServerParticipantConfiguration.hpp
 */

#ifndef _DDSROUTERCORE_CONFIGURATION_PARTICIPANT_DISCOVERYSERVERPARTICIPANTCONFIGURATION_HPP_
#define _DDSROUTERCORE_CONFIGURATION_PARTICIPANT_DISCOVERYSERVERPARTICIPANTCONFIGURATION_HPP_

#include <ddsrouter_core/configuration/participant/SimpleParticipantConfiguration.hpp>
#include <ddsrouter_core/library/library_dll.h>
#include <ddsrouter_core/types/security/tls/TlsConfiguration.hpp>
#include <ddsrouter_core/types/address/Address.hpp>
#include <ddsrouter_core/types/address/DiscoveryServerConnectionAddress.hpp>
#include <ddsrouter_core/types/dds/DomainId.hpp>
#include <ddsrouter_core/types/dds/GuidPrefix.hpp>
#include <ddsrouter_core/types/participant/ParticipantId.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace configuration {

/**
 * This class joins Discovery Server Participant Configuration features and gives methods to interact with it.
 */
class DiscoveryServerParticipantConfiguration : public SimpleParticipantConfiguration
{
public:

    /**
     * @brief Single constructor
     *
     * @param id Participant ID
     * @param server_guid_prefix Server GUID prefix
     * @param listening_addresses Listening addresses
     * @param connection_addresses Connection addresses
     * @param domain_id Domain ID
     * @param tls_configuration TLS configuration
     */
    DDSROUTER_CORE_DllAPI DiscoveryServerParticipantConfiguration(
            const types::ParticipantId& id,
            const types::GuidPrefix& discovery_server_guid_prefix,
            const std::set<types::Address>& listening_addresses,
            const std::set<types::DiscoveryServerConnectionAddress>& connection_addresses,
            const types::DomainId domain_id = types::DEFAULT_DOMAIN_ID,
            const types::security::TlsConfiguration& tls_configuration = types::security::TlsConfiguration());

    //! Discovery server GUID prefix getter
    DDSROUTER_CORE_DllAPI const types::GuidPrefix& discovery_server_guid_prefix() const noexcept;

    //! Listening addresses getter
    DDSROUTER_CORE_DllAPI const std::set<types::Address>& listening_addresses() const noexcept;

    //! Connection addresses getter
    DDSROUTER_CORE_DllAPI const std::set<types::DiscoveryServerConnectionAddress>& connection_addresses() const noexcept;

    //! TLS configuration getter
    DDSROUTER_CORE_DllAPI const types::security::TlsConfiguration& tls_configuration() const noexcept;

protected:

    //! GuidPrefix
    const types::GuidPrefix discovery_server_guid_prefix_;

    //! Listening addresses
    const std::set<types::Address> listening_addresses_;

    //! Connection addresses
    const std::set<types::DiscoveryServerConnectionAddress> connection_addresses_;

    //! TLS configuration
    const types::security::TlsConfiguration tls_configuration_;
};

} /* namespace configuration */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_CONFIGURATION_PARTICIPANT_DISCOVERYSERVERPARTICIPANTCONFIGURATION_HPP_ */
