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
#include <ddsrouter_core/types/participant/ParticipantKind.hpp>

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

    // TODO
    DDSROUTER_CORE_DllAPI DiscoveryServerParticipantConfiguration(
            const types::ParticipantId& id,
            const types::GuidPrefix& discovery_server_guid_prefix,
            const std::set<types::Address>& listening_addresses,
            const std::set<types::DiscoveryServerConnectionAddress>& connection_addresses,
            const types::ParticipantKind& kind = types::ParticipantKind::local_discovery_server,
            std::shared_ptr<types::security::TlsConfiguration> tls_configuration =
            std::make_shared<types::security::TlsConfiguration>(),
            const types::DomainId& domain_id = DEFAULT_DS_DOMAIN_ID_);

    // TODO
    DDSROUTER_CORE_DllAPI DiscoveryServerParticipantConfiguration(
            const types::ParticipantId& id,
            const types::GuidPrefix& discovery_server_guid_prefix,
            const std::set<types::Address>& listening_addresses,
            const std::set<types::DiscoveryServerConnectionAddress>& connection_addresses,
            const types::DomainId& domain_id,
            const types::ParticipantKind& kind = types::ParticipantKind::local_discovery_server,
            std::shared_ptr<types::security::TlsConfiguration> tls_configuration =
            std::make_shared<types::security::TlsConfiguration>());

    DDSROUTER_CORE_DllAPI types::GuidPrefix discovery_server_guid_prefix() const noexcept;

    DDSROUTER_CORE_DllAPI std::set<types::Address> listening_addresses() const noexcept;

    DDSROUTER_CORE_DllAPI std::set<types::DiscoveryServerConnectionAddress> connection_addresses() const noexcept;

    DDSROUTER_CORE_DllAPI bool tls_active() const noexcept;

    DDSROUTER_CORE_DllAPI std::shared_ptr<types::security::TlsConfiguration> tls_configuration() const;

    DDSROUTER_CORE_DllAPI virtual bool is_valid(
            utils::Formatter& error_msg) const noexcept override;

    DDSROUTER_CORE_DllAPI bool operator ==(
            const DiscoveryServerParticipantConfiguration& other) const noexcept;

    DDSROUTER_CORE_DllAPI static types::DomainId default_domain_id() noexcept;

protected:

    types::GuidPrefix discovery_server_guid_prefix_;
    std::set<types::Address> listening_addresses_;
    std::set<types::DiscoveryServerConnectionAddress> connection_addresses_;
    std::shared_ptr<types::security::TlsConfiguration> tls_configuration_;

    DDSROUTER_CORE_DllAPI static const types::DomainId DEFAULT_DS_DOMAIN_ID_; // 66
};

} /* namespace configuration */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_CONFIGURATION_PARTICIPANT_DISCOVERYSERVERPARTICIPANTCONFIGURATION_HPP_ */
