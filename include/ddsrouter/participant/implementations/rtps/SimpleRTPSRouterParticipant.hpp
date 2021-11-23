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
 * @file SimpleRTPSRouterParticipant.hpp
 */

#ifndef _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_AUX_SIMPLERTPSPARTICIPANT_HPP_
#define _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_AUX_SIMPLERTPSPARTICIPANT_HPP_

#include <ddsrouter/configuration/SimpleRTPSParticipantConfiguration.hpp>
#include <ddsrouter/participant/implementations/auxiliar/BaseParticipant.hpp>
#include <ddsrouter/reader/implementations/rtps/RTPSRouterReader.hpp>
#include <ddsrouter/writer/implementations/rtps/RTPSRouterWriter.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * TODO
 */
class SimpleRTPSRouterParticipant : public BaseParticipant<SimpleRTPSParticipantConfiguration>
{
public:

    SimpleRTPSRouterParticipant(
            const SimpleRTPSParticipantConfiguration& participant_configuration,
            std::shared_ptr<PayloadPool> payload_pool,
            std::shared_ptr<DiscoveryDatabase> discovery_database);

protected:

    std::shared_ptr<IWriter> create_writer_(
            RealTopic topic) override;

    std::shared_ptr<IReader> create_reader_(
            RealTopic topic) override;

    void delete_writer_(
            std::shared_ptr<IWriter> writer) noexcept override;

    void delete_reader_(
            std::shared_ptr<IReader> reader) noexcept override;

    //! Mutex that guards every access to the RTPS Participant
    mutable std::recursive_mutex rtps_mutex_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_AUX_SIMPLERTPSPARTICIPANT_HPP_ */
