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
 * @file DummyParticipant.hpp
 */

#ifndef __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_DUMMYPARTICIPANT_HPP_
#define __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_DUMMYPARTICIPANT_HPP_

#include <ddsrouter_core/configuration/participant/ParticipantConfiguration.hpp>

#include <participant/implementations/auxiliar/BaseParticipant.hpp>
#include <reader/implementations/auxiliar/DummyReader.hpp>
#include <writer/implementations/auxiliar/DummyWriter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Concrete Participant that allows to simulate a real remote network.
 *
 * This Participant includes methods that allow to simulate the reception of a message
 * and to get the messages that should have been sent.
 * For this, a static map is created so it can store all the DummyParticipants created and have access to their methods.
 *
 * This Participant is used for Testing, as it could mock a DDS real network.
 */
class DummyParticipant : public BaseParticipant
{
public:

    /**
     * @brief Construct a new Dummy Participant object
     *
     * It uses the \c BaseParticipant constructor.
     * Apart from BaseParticipant, it adds this new object to a static variable so it could be reached from outside
     * the DDSRouter.
     */
    DummyParticipant(
            std::shared_ptr<configuration::ParticipantConfiguration> participant_configuration,
            std::shared_ptr<PayloadPool> payload_pool,
            std::shared_ptr<DiscoveryDatabase> discovery_database);

    /**
     * @brief Destroy the Dummy Participant object
     *
     * Remove its reference from the static map
     */
    virtual ~DummyParticipant();

    /**
     * @brief Simulate that this Participant has discovered a new endpoint
     *
     * @param new_endpoint : Endpoint discovered
     */
    void simulate_discovered_endpoint(
            const types::Endpoint& new_endpoint);

    /**
     * @brief Get the discovered endpoint object referring to \c guid
     *
     * Search this guid in the endpoints in the Discovery Database
     *
     * @param guid : \c Guid of the Endpoint to look for
     * @return Endpoint with this guid
     */
    types::Endpoint get_discovered_endpoint(
            const types::Guid& guid) const;

    /**
     * @brief Simulate that the reader of the topic has received a message
     *
     * @param topic : Topic that refers to the Reader that should forward this data.
     * @param data : data received
     */
    void simulate_data_reception(
            types::DdsTopic topic,
            DummyDataReceived data);

    /**
     * @brief Get the data that has arrived to this Writer in order to send it.
     *
     * @param topic : Topic that refers to the Writer that should have sent this data.
     * @return Vector of all the data that the Writer should have sent.
     */
    std::vector<DummyDataStored> get_data_that_should_have_been_sent(
            types::DdsTopic topic);

    /**
     * @brief Make the thread wait until message \c n has arrived to Writer in topic \c topic
     *
     * @param topic : Topic that refers to the Writer that should have sent this data.
     * @param [in] n : wait until data \c n has arrived and simulated to be sent
     */
    void wait_until_n_data_sent(
            types::DdsTopic topic,
            uint16_t n) const noexcept;

    /**
     * @brief Get a DummyParticipant by ID
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
    static DummyParticipant* get_participant(
            types::ParticipantId id);

protected:

    //! Override create_writer_() BaseParticipant method
    std::shared_ptr<IWriter> create_writer_(
            types::DdsTopic topic) override;

    //! Override create_reader_() BaseParticipant method
    std::shared_ptr<IReader> create_reader_(
            types::DdsTopic topic) override;

    // Specific enable/disable do not need to be implemented

    //! Mutex to guard the static map \c participants_
    static std::mutex static_mutex_;

    //! Static map that stores every DummyParticipant running
    static std::map<types::ParticipantId, DummyParticipant*> participants_;
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_DUMMYPARTICIPANT_HPP_ */
