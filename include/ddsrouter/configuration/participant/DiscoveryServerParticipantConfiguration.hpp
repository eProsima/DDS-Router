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

#ifndef _DDSROUTER_CONFIGURATION_DISCOVERYSERVERPARTICIPANTCONFIGURATION_HPP_
#define _DDSROUTER_CONFIGURATION_DISCOVERYSERVERPARTICIPANTCONFIGURATION_HPP_

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
 * This class joins Discovery Server Participant Configuration features and give methods to interact with it.
 */
class DiscoveryServerParticipantConfiguration : public SimpleParticipantConfiguration
{
public:

    // TODO
    DiscoveryServerParticipantConfiguration(
            const ParticipantId& id,
            const GuidPrefix& discovery_server_guid_prefix,
            const std::set<std::shared_ptr<Address>>& listening_addresses,
            const std::set<std::shared_ptr<DiscoveryServerConnectionAddress>>& connection_addresses,
            const ParticipantKind& type = ParticipantKind::LOCAL_DISCOVERY_SERVER,
            const security::TlsConfiguration& tls_configuration = security::TlsConfiguration(),
            const DomainId& domain_id = DEFAULT_DS_DOMAIN_ID_);

    GuidPrefix discovery_server_guid_prefix() const noexcept;

    std::set<std::shared_ptr<Address>> listening_addresses() const noexcept;

    std::set<std::shared_ptr<DiscoveryServerConnectionAddress>> connection_addresses() const noexcept;

    bool tls_active() const noexcept;

    security::TlsConfiguration tls_configuration() const;

    virtual bool is_valid() const noexcept override;

    bool operator ==(
            const DiscoveryServerParticipantConfiguration& other) const noexcept;

protected:

    GuidPrefix discovery_server_guid_;
    std::set<std::shared_ptr<Address>> listening_addresses_;
    std::set<std::shared_ptr<DiscoveryServerConnectionAddress>> connection_addresses_;
    const security::TlsConfiguration& tls_configuration_;

    static const DomainId DEFAULT_DS_DOMAIN_ID_; // 66
};

} /* namespace configuration */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_CONFIGURATION_DISCOVERYSERVERPARTICIPANTCONFIGURATION_HPP_ */
