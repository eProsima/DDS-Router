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

#include <cpp_utils/macros/custom_enumeration.hpp>
#include <cpp_utils/enum/EnumBuilder.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

ENUMERATION_BUILDER(
    ParticipantKind,
    simple,
    initial_peers,
    discovery_server,
    echo,
    xml
    );

eProsima_ENUMERATION_BUILDER(
    ParticipantKindBuilder,
    ParticipantKind,
                {
                    { ParticipantKind::simple COMMA { "local" COMMA "simple" } } COMMA
                    { ParticipantKind::initial_peers COMMA {"wan" COMMA "router" COMMA "initial-peers"} } COMMA
                    { ParticipantKind::discovery_server COMMA {"discovery-server" COMMA "ds" COMMA "local-ds" COMMA "local-discovery-server" COMMA "wan-ds" COMMA "wan-discovery-server"} } COMMA
                    { ParticipantKind::echo COMMA {"echo"} } COMMA
                    { ParticipantKind::xml COMMA {"xml" COMMA "XML"} }
                }
    );

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
