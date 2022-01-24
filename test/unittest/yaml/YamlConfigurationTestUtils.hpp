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
 * @file YamlConfigurationTestUtils.hpp
 */

#ifndef _DDSROUTER_TEST_UNITTEST_YAML_YAMLCONFIGURATIONTESTUTILS_HPP_
#define _DDSROUTER_TEST_UNITTEST_YAML_YAMLCONFIGURATIONTESTUTILS_HPP_

#include <ddsrouter/yaml/Yaml.hpp>

namespace eprosima {
namespace ddsrouter {
namespace yaml {
namespace test {

template <typename T>
struct YamlField
{
    YamlField() : present(false) {}
    YamlField(T arg_value) : value(arg_value) , present(true) {}

    bool present;
    T value;
};

template <typename T>
void add_field_to_yaml(
    Yaml& yml,
    const YamlField<T>& field,
    const std::string& tag)
{
    if (field.present)
    {
        yml[tag] = field.value;
    }
}

} /* namespace test */
} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TEST_UNITTEST_YAML_YAMLCONFIGURATIONTESTUTILS_HPP_ */
