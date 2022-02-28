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

#ifndef _DDSROUTER_CONFIGURATION_PARTICIPANT_DISCOVERYSERVERPARTICIPANTCONFIGURATION_HPP_
#define _DDSROUTER_CONFIGURATION_PARTICIPANT_DISCOVERYSERVERPARTICIPANTCONFIGURATION_HPP_

#include <ddsrouter/configuration/participant/SimpleParticipantConfiguration.hpp>
#include <ddsrouter/security/tls/TlsConfiguration.hpp>
#include <ddsrouter/types/address/Address.hpp>
#include <ddsrouter/types/address/DiscoveryServerConnectionAddress.hpp>
#include <ddsrouter/types/endpoint/DomainId.hpp>
#include <ddsrouter/types/endpoint/GuidPrefix.hpp>
#include <ddsrouter/types/participant/ParticipantKind.hpp>

namespace eprosima {
namespace ddsrouter {
namespace configuration {

/**
 * This class joins Discovery Server Participant Configuration features and gives methods to interact with it.
 */
class DiscoveryServerParticipantConfiguration : public SimpleParticipantConfiguration
{
public:

    // TODO
    DiscoveryServerParticipantConfiguration(
            const ParticipantId& id,
            const GuidPrefix& discovery_server_guid_prefix,
            const std::set<Address>& listening_addresses,
            const std::set<DiscoveryServerConnectionAddress>& connection_addresses,
            const ParticipantKind& kind = ParticipantKind::LOCAL_DISCOVERY_SERVER,
            std::shared_ptr<security::TlsConfiguration> tls_configuration =
            std::make_shared<security::TlsConfiguration>(),
            const DomainId& domain_id = DEFAULT_DS_DOMAIN_ID_);

    // TODO
    DiscoveryServerParticipantConfiguration(
            const ParticipantId& id,
            const GuidPrefix& discovery_server_guid_prefix,
            const std::set<Address>& listening_addresses,
            const std::set<DiscoveryServerConnectionAddress>& connection_addresses,
            const DomainId& domain_id,
            const ParticipantKind& kind = ParticipantKind::LOCAL_DISCOVERY_SERVER,
            std::shared_ptr<security::TlsConfiguration> tls_configuration =
            std::make_shared<security::TlsConfiguration>());

    GuidPrefix discovery_server_guid_prefix() const noexcept;

    std::set<Address> listening_addresses() const noexcept;

    std::set<DiscoveryServerConnectionAddress> connection_addresses() const noexcept;

    bool tls_active() const noexcept;

    std::shared_ptr<security::TlsConfiguration> tls_configuration() const;

    virtual bool is_valid(
            utils::Formatter& error_msg) const noexcept override;

    bool operator ==(
            const DiscoveryServerParticipantConfiguration& other) const noexcept;

    static DomainId default_domain_id() noexcept;

protected:

    GuidPrefix discovery_server_guid_prefix_;
    std::set<Address> listening_addresses_;
    std::set<DiscoveryServerConnectionAddress> connection_addresses_;
    std::shared_ptr<security::TlsConfiguration> tls_configuration_;

    static const DomainId DEFAULT_DS_DOMAIN_ID_; // 66
};

} /* namespace configuration */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_CONFIGURATION_PARTICIPANT_DISCOVERYSERVERPARTICIPANTCONFIGURATION_HPP_ */
