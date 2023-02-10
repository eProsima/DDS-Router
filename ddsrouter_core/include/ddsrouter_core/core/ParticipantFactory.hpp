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

#include <ddspipe_core/interface/IParticipant.hpp>
#include <ddspipe_core/efficiency/payload/PayloadPool.hpp>
#include <ddspipe_core/dynamic/DiscoveryDatabase.hpp>

#include <ddspipe_participants/configuration/ParticipantConfiguration.hpp>

#include <ddsrouter_core/types/ParticipantKind.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

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
    static std::shared_ptr<ddspipe::core::IParticipant> create_participant(
            const types::ParticipantKind& kind,
            const std::shared_ptr<ddspipe::participants::ParticipantConfiguration>& participant_configuration,
            const std::shared_ptr<ddspipe::core::PayloadPool>& payload_pool,
            const std::shared_ptr<ddspipe::core::DiscoveryDatabase>& discovery_database);
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
