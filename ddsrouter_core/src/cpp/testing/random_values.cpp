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

#include <cpp_utils/exception/InconsistencyException.hpp>

#include <ddspipe_core/testing/random_values.hpp>

#include <ddspipe_participants/testing/random_values.hpp>
#include <ddspipe_participants/configuration/DiscoveryServerParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/EchoParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/InitialPeersParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/ParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/SimpleParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/XmlParticipantConfiguration.hpp>

#include <ddsrouter_core/testing/random_values.hpp>
#include <ddsrouter_core/types/ParticipantKind.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace testing {

using namespace eprosima::ddspipe::core::testing;
using namespace eprosima::ddspipe::core::types;
using namespace eprosima::ddspipe::participants;
using namespace eprosima::ddspipe::participants::testing;
using namespace eprosima::ddspipe::participants::types;
using namespace eprosima::ddsrouter::core::types;

std::shared_ptr<ParticipantConfiguration> random_participant_configuration(
        ParticipantKind kind,
        unsigned int seed /* = 0 */)
{
    ParticipantId id = random_participant_id(seed);

    switch (kind)
    {
        case ParticipantKind::simple:
        {
            auto c = std::make_shared<SimpleParticipantConfiguration>();
            c->id = id;
            c->domain = random_domain(seed);
            return c;
        }

        case ParticipantKind::discovery_server:
        {
            auto c = std::make_shared<DiscoveryServerParticipantConfiguration>();
            c->id = id;
            c->domain = random_domain(seed);
            c->discovery_server_guid_prefix = random_guid_prefix(seed);
            c->connection_addresses = {random_connection_address(seed)};
            return c;
        }

        case ParticipantKind::initial_peers:

        {
            auto c = std::make_shared<InitialPeersParticipantConfiguration>();
            c->id = id;
            c->domain = random_domain(seed);
            c->listening_addresses = {random_address(seed)};
            return c;
        }

        case ParticipantKind::echo:
        {
            auto c = std::make_shared<EchoParticipantConfiguration>();
            c->id = id;
            return c;
        }

        case ParticipantKind::xml:
        {
            auto c = std::make_shared<XmlParticipantConfiguration>();
            c->id = id;
            return c;
        }

        default:
            throw eprosima::utils::InconsistencyException("No valid kind");
    }
}

ParticipantKind random_participant_kind(
        unsigned int seed /* = 0 */)
{
    return VALUES_ParticipantKind[seed % N_VALUES_ParticipantKind];
}

} /* namespace testing */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
