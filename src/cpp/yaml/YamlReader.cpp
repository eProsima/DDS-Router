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

#include <ddsrouter/types/address/Address.hpp>
#include <ddsrouter/types/address/DiscoveryServerConnectionAddress.hpp>
#include <ddsrouter/types/endpoint/DomainId.hpp>
#include <ddsrouter/types/endpoint/GuidPrefix.hpp>
#include <ddsrouter/types/topic/RealTopic.hpp>
#include <ddsrouter/types/topic/WildcardTopic.hpp>
#include <ddsrouter/yaml/YamlReader.hpp>
#include <ddsrouter/yaml/yaml_configuration_tags.hpp>

namespace eprosima {
namespace ddsrouter {
namespace yaml {

bool YamlReader::is_tag_present(const Yaml& yml, const TagType& tag)
{
    if (!yml.IsMap())
    {
        throw ConfigurationException(
            utils::Formatter() << "Trying to find a tag: <" << tag << "> in a not yaml object map.");
    }

    return (yml[tag]);
}

Yaml YamlReader::get_value_in_tag(const Yaml& yml, const TagType& tag)
{
    if (yml[tag])
    {
        return yml[tag];
    }
    else
    {
        throw ConfigurationException(
            utils::Formatter() << "Required tag not found: <" << tag << ">.");
    }
}

} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */
