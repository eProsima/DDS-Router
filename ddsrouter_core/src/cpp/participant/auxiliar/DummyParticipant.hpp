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

#include <participant/auxiliar/GenericParticipant.ipp>
#include <writer/auxiliar/DummyWriter.hpp>
#include <reader/auxiliar/DummyReader.hpp>


namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Concrete Participant that allows to simulate a real remote network mocking RTPS participants..
 *
 * This Participant includes methods that allow to mock the reception and forward of a message
 *
 * These Participants are used for Testing, as it could mock a DDS real network.
 */
class DummyParticipant : public GenericParticipant<types::ParticipantKind::dummy>
{
public:

    //! Using parent class constructors
    using GenericParticipant<types::ParticipantKind::dummy>::GenericParticipant;

    using GenericParticipant<types::ParticipantKind::dummy>::create_writer_;
    using GenericParticipant<types::ParticipantKind::dummy>::create_reader_;

    /**
     * @brief Return Reader associatd to Topic
     *
     * @param topic : Input topic
     */
    DummyReader* get_topic_reader(
            const types::RealTopic& topic);

    /**
     * @brief Return Writer associatd to Topic
     *
     * @param topic : Input topic
     */
    DummyWriter* get_topic_writer(
            const types::RealTopic& topic);
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_DUMMYPARTICIPANT_HPP_ */
