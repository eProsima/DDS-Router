// Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#pragma once

#include <ddspipe_participants/testing/random_values.hpp>

#include <ddspipe_yaml/Yaml.hpp>
#include <ddspipe_yaml/yaml_configuration_tags.hpp>
#include <ddspipe_yaml/testing/generate_yaml.hpp>

#include <ddsrouter_core/types/ParticipantKind.hpp>

namespace eprosima {
namespace ddsrouter {
namespace yaml {
namespace testing {

void participantkind_to_yaml(
        Yaml& yml,
        const ddsrouter::core::types::ParticipantKind& kind)
{
    ddspipe::yaml::testing::add_field_to_yaml(
        yml,
        ddspipe::yaml::testing::YamlField<std::string>(to_string(kind)),
        ddspipe::yaml::PARTICIPANT_KIND_TAG);
}

} /* namespace testing */
} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */
