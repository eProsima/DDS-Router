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
 * @file ParticipantFactory.hpp
 */

#ifndef _DDSROUTER_PARTICIPANT_DDS_ROUTERPARTICIPANTFACTORY_HPP_
#define _DDSROUTER_PARTICIPANT_DDS_ROUTERPARTICIPANTFACTORY_HPP_

#include <ddsrouter/configuration/participant/ParticipantConfiguration.hpp>
#include <ddsrouter/participant/IParticipant.hpp>

namespace eprosima {
namespace ddsrouter {

class ParticipantFactory
{
public:

    /**
     * @brief Create a participant object of the kind specified in the configuration.
     *
     * @throw ConfigurationException : in case the configuration or participant kind is incorrect
     *
     * @param [in] participant_configuration : Participant Configuration that will retrieve the id and kind of the
     *      participant that will be created, and its Configuration.
     * @param [in] payload : Common Payload pool to create the Participant
     * @param [in] discovery_database : Common Discovery Database to create the Participant
     * @return new Participant
     */
    std::shared_ptr<IParticipant> create_participant(
            std::shared_ptr<configuration::ParticipantConfiguration> participant_configuration,
            std::shared_ptr<PayloadPool> payload,
            std::shared_ptr<DiscoveryDatabase> discovery_database);

    /**
     * @brief Delete correctly a Participant
     *
     * When calling this method, the participant shared ptr must only be referenced here,
     * so this method should be able to destroy completely the Participant
     *
     * @param participant : participant to be deleted
     */
    void remove_participant(
            std::shared_ptr<IParticipant> participant);
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_PARTICIPANT_DDS_ROUTERPARTICIPANTFACTORY_HPP_ */
