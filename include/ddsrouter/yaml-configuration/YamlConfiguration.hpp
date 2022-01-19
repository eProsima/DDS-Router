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
 * @file DDSRouterConfiguration.hpp
 */

#ifndef _DDSROUTER_YAMLCONFIGURATION_YAMLCONFIGURATION_HPP_
#define _DDSROUTER_YAMLCONFIGURATION_YAMLCONFIGURATION_HPP_

#include <ddsrouter/configuration/DDSRouterConfiguration.hpp>
#include <ddsrouter/configuration/participant/DiscoveryServerParticipantConfiguration.hpp>
#include <ddsrouter/configuration/participant/SimpleParticipantConfiguration.hpp>
#include <ddsrouter/configuration/participant/DiscoveryServerParticipantConfiguration.hpp>
#include <ddsrouter/configuration/DDSRouterConfiguration.hpp>
#include <ddsrouter/configuration/DDSRouterConfiguration.hpp>
#include <ddsrouter/types/address/Address.hpp>
#include <ddsrouter/types/address/DiscoveryServerConnectionAddress.hpp>
#include <ddsrouter/types/endpoint/GuidPrefix.hpp>
#include <ddsrouter/types/endpoint/DomainId.hpp>
#include <ddsrouter/types/topic/FilterTopic.hpp>
#include <ddsrouter/types/topic/RealTopic.hpp>
#include <ddsrouter/yaml-configuration/Yaml.hpp>

namespace eprosima {
namespace ddsrouter {
namespace yaml {

/**
 * TODO
 */
class YamlDDSRouterConfiguration
{
public:
    static configuration::DDSRouterConfiguration ddsrouter_configuration(const Yaml& yaml);

    // TODO load yaml version and work differently depending on the version
    // static version configuration_version(const Yaml& yaml);

    static std::set<std::shared_ptr<configuration::ParticipantConfiguration>> participants_configurations(const Yaml& yaml);

    static std::set<std::shared_ptr<FilterTopic>> allowlist(const Yaml& yaml);
    static std::set<std::shared_ptr<FilterTopic>> blocklist(const Yaml& yaml);
    static std::set<std::shared_ptr<RealTopic>> builtin_topics(const Yaml& yaml);

protected:

    static std::set<std::shared_ptr<FilterTopic>> generic_filter_topic(
        const Yaml& yaml,
        std::string list_tag);
};

class YamlElementConfiguration
{
public:
    static DomainId domain_id(
            const Yaml& yaml,
            bool required = true,
            bool default_value = false);
    static GuidPrefix guid_prefix(const Yaml& yaml);

    static Address address(const Yaml& yaml);
    static DiscoveryServerConnectionAddress discovery_server_connection_address(const Yaml& yaml);

    static std::shared_ptr<FilterTopic> filter_topic(const Yaml& yaml);
    static std::shared_ptr<RealTopic> real_topic(const Yaml& yaml);

};

class YamlParticipantConfiguration
{
public:

    static ParticipantId participant_id(const Yaml& yam);
    static ParticipantType participant_type(const Yaml& yam);

    static std::shared_ptr<configuration::ParticipantConfiguration> participant_configuration_factory(const Yaml& yaml);

    static configuration::ParticipantConfiguration std_participant_configuration(
        const Yaml& yaml,
        ParticipantType type);

    static configuration::SimpleParticipantConfiguration simple_participant_configuration(
        const Yaml& yaml,
        ParticipantType type);

    static configuration::DiscoveryServerParticipantConfiguration discovery_server_participant_configuration(
        const Yaml& yaml,
        ParticipantType type);
};

} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_YAMLCONFIGURATION_YAMLCONFIGURATION_HPP_ */
