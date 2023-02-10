// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file YamlReader.cpp
 *
 */

#include <cpp_utils/Log.hpp>
#include <cpp_utils/utils.hpp>

#include <ddspipe_core/types/dds/DomainId.hpp>
#include <ddspipe_core/types/dds/GuidPrefix.hpp>
#include <ddspipe_core/types/participant/ParticipantId.hpp>
#include <ddspipe_core/types/topic/dds/DdsTopic.hpp>
#include <ddspipe_core/types/topic/filter/WildcardDdsFilterTopic.hpp>

#include <ddspipe_participants/types/address/Address.hpp>
#include <ddspipe_participants/types/address/DiscoveryServerConnectionAddress.hpp>
#include <ddspipe_participants/types/security/tls/TlsConfiguration.hpp>

#include <ddspipe_participants/configuration/DiscoveryServerParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/InitialPeersParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/ParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/EchoParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/SimpleParticipantConfiguration.hpp>

#include <ddspipe_yaml/Yaml.hpp>
#include <ddspipe_yaml/YamlReader.hpp>
#include <ddspipe_yaml/yaml_configuration_tags.hpp>

namespace eprosima {
namespace ddspipe {
namespace yaml {

using namespace eprosima::ddspipe::core::types;
using namespace eprosima::ddspipe::participants::types;

/************************
* GENERIC              *
************************/

bool YamlReader::is_tag_present(
        const Yaml& yml,
        const TagType& tag)
{
    if (!yml.IsMap() && !yml.IsNull())
    {
        throw eprosima::utils::ConfigurationException(
                  utils::Formatter() << "Trying to find a tag: <" << tag << "> in a not yaml object map.");
    }

    // Explicit conversion to avoid windows format warning
    // This method performace is the same as only retrieving bool
    return (yml[tag]) ? true : false;
}

Yaml YamlReader::get_value_in_tag(
        const Yaml& yml,
        const TagType& tag)
{
    if (is_tag_present(yml, tag))
    {
        return yml[tag];
    }
    else
    {
        throw eprosima::utils::ConfigurationException(
                  utils::Formatter() << "Required tag not found: <" << tag << ">.");
    }
}

/************************
* STANDARD             *
************************/

template <>
int YamlReader::get<int>(
        const Yaml& yml,
        const YamlReaderVersion version /* version */)
{
    return get_scalar<int>(yml);
}

template <>
unsigned int YamlReader::get<unsigned int>(
        const Yaml& yml,
        const YamlReaderVersion version /* version */)
{
    return get_scalar<unsigned int>(yml);
}

template <>
bool YamlReader::get<bool>(
        const Yaml& yml,
        const YamlReaderVersion version /* version */)
{
    return get_scalar<bool>(yml);
}

template <>
std::string YamlReader::get<std::string>(
        const Yaml& yml,
        const YamlReaderVersion version /* version */)
{
    return get_scalar<std::string>(yml);
}

} /* namespace yaml */
} /* namespace ddspipe */
} /* namespace eprosima */
