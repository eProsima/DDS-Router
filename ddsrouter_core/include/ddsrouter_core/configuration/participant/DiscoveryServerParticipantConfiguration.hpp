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
struct DiscoveryServerParticipantConfiguration : public SimpleParticipantConfiguration
{

    /////////////////////////
    // CONSTRUCTORS
    /////////////////////////

    DDSROUTER_CORE_DllAPI DiscoveryServerParticipantConfiguration() = default;

    DDSROUTER_CORE_DllAPI DiscoveryServerParticipantConfiguration(
            const types::ParticipantId& id,
            const types::ParticipantKind& kind,
            const bool is_repeater,
            const types::DomainId& domain_id,
            const types::GuidPrefix& discovery_server_guid_prefix,
            const std::set<types::Address>& listening_addresses,
            const std::set<types::DiscoveryServerConnectionAddress>& connection_addresses,
            const types::security::TlsConfiguration& tls_configuration);

    /////////////////////////
    // METHODS
    /////////////////////////

    DDSROUTER_CORE_DllAPI virtual bool is_valid(
            utils::Formatter& error_msg) const noexcept override;

    DDSROUTER_CORE_DllAPI bool operator ==(
            const DiscoveryServerParticipantConfiguration& other) const noexcept;

    /////////////////////////
    // VARIABLES
    /////////////////////////

    types::GuidPrefix discovery_server_guid_prefix = types::GuidPrefix();

    std::set<types::Address> listening_addresses = {};

    std::set<types::DiscoveryServerConnectionAddress> connection_addresses = {};

    types::security::TlsConfiguration tls_configuration = types::security::TlsConfiguration();
};

} /* namespace configuration */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_CONFIGURATION_PARTICIPANT_DISCOVERYSERVERPARTICIPANTCONFIGURATION_HPP_ */
