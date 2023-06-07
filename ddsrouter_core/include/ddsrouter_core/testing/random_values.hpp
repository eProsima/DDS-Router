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

#pragma once

#include <ddspipe_participants/configuration/ParticipantConfiguration.hpp>

#include <ddsrouter_core/types/ParticipantKind.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace testing {

std::shared_ptr<ddspipe::participants::ParticipantConfiguration> random_participant_configuration(
        types::ParticipantKind kind,
        unsigned int seed = 0);

types::ParticipantKind random_participant_kind(
        unsigned int seed = 0);

} /* namespace testing */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
