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
 * @file MockParticipant.hpp
 */

#ifndef __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_MOCKPARTICIPANT_HPP_
#define __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_MOCKPARTICIPANT_HPP_

#include <ddsrouter_core/configuration/participant/ParticipantConfiguration.hpp>

#include <participant/implementations/auxiliar/BaseParticipant.hpp>
#include <reader/implementations/auxiliar/MockReader.hpp>
#include <writer/implementations/auxiliar/MockWriter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Concrete Participant that allows to simulate a real remote network in an efficient way.
 *
 * This Participant includes methods that allow to simulate the reception of a message
 * and to get the messages that should have been sent.
 * For this, a static map is created so it can store all the MockParticipants created and have access to their methods.
 *
 * This Participant is used for Testing, as it could mock a DDS real network.
 */
class MockParticipant : public BaseParticipant<configuration::ParticipantConfiguration>
{
public:

    /**
     * @brief Construct a new Mock Participant object
     *
     * It uses the \c BaseParticipant constructor.
     * Apart from BaseParticipant, it adds this new object to a static variable so it could be reached from outside
     * the DDSRouter.
     */
    MockParticipant(
            const configuration::ParticipantConfiguration participant_configuration,
            std::shared_ptr<PayloadPool> payload_pool,
            std::shared_ptr<DiscoveryDatabase> discovery_database);

    /**
     * @brief Destroy the Mock Participant object
     *
     * Remove its reference from the static map
     */
    virtual ~MockParticipant();

    MockReader* get_reader(
            const types::RealTopic& topic);

    MockWriter* get_writer(
            const types::RealTopic& topic);

    /**
     * @brief Get a MockParticipant by ID
     *
     * By this reference, the internal writers and readers of the Participant could be accessed in order to simulate
     * the reception of data, and to get the data that must be sent.
     * Use methods \c simulate_data_reception and \c get_data_that_should_have_been_sent for this purpose.
     *
     * @param id : Id of the participant
     *
     * @return raw pointer to the participant
     * @warning Do not remove this ptr
     */
    static MockParticipant* get_participant(
            types::ParticipantId id);

protected:

    //! Override create_writer_() BaseParticipant method
    std::shared_ptr<IWriter> create_writer_(
            types::RealTopic topic) override;

    //! Override create_reader_() BaseParticipant method
    std::shared_ptr<IReader> create_reader_(
            types::RealTopic topic) override;

    // Specific enable/disable do not need to be implemented

    //! Mutex to guard the static map \c participants_
    static std::mutex static_mutex_;

    //! Static map that stores every MockParticipant running
    static std::map<types::ParticipantId, MockParticipant*> participants_;
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_MOCKPARTICIPANT_HPP_ */
