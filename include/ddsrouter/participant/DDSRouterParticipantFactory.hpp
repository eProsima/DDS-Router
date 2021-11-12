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
 * @file DDSRouterParticipantFactory.hpp
 */

#ifndef _DDS_ROUTER_PARTICIPANT_DDS_ROUTERPARTICIPANTFACTORY_HPP_
#define _DDS_ROUTER_PARTICIPANT_DDS_ROUTERPARTICIPANTFACTORY_HPP_

#include <ddsrouter/participant/IDDSRouterParticipant.hpp>
#include <ddsrouter/types/RawConfiguration.hpp>
#include <ddsrouter/types/ReturnCode.hpp>

namespace eprosima {
namespace ddsrouter {

class DDSRouterParticipantFactory
{
public:

    DDSRouterParticipantFactory() = default;

    virtual ~DDSRouterParticipantFactory();

    std::shared_ptr<IDDSRouterParticipant> create_participant(
            ParticipantId id,
            RawConfiguration participant_configuration,
            std::shared_ptr<PayloadPool> payload,
            std::shared_ptr<DiscoveryDatabase> discovery_database);
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDS_ROUTER_PARTICIPANT_DDS_ROUTERPARTICIPANTFACTORY_HPP_ */
