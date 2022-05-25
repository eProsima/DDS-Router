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
 * @file EmptyParticipant.hpp
 */

#ifndef __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_VOIDPARTICIPANT_HPP_
#define __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_VOIDPARTICIPANT_HPP_

#include <ddsrouter_core/configuration/participant/ParticipantConfiguration.hpp>
#include <participant/IParticipant.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Participant that has an empty implementation.
 * It does not discover anything.
 *
 * Writer: EmptyWriter
 * Reader: EmptyReader
 */
class EmptyParticipant : public IParticipant
{
public:

    //! Constructor with Id of this participant
    EmptyParticipant(
            const types::ParticipantId& id_);

    //! Override id() IParticipant method
    types::ParticipantId id() const noexcept override;

    //! Override kind() IParticipant method
    types::ParticipantKind kind() const noexcept override;

    //! Override create_writer() IParticipant method
    std::shared_ptr<IWriter> create_writer(
            types::RealTopic topic) override;

    //! Override create_reader() IParticipant method
    std::shared_ptr<IReader> create_reader(
            types::RealTopic topic) override;

    //! Override delete_writer() IParticipant method
    void delete_writer(
            std::shared_ptr<IWriter> writer) noexcept override;

    //! Override delete_reader() IParticipant method
    void delete_reader(
            std::shared_ptr<IReader> reader) noexcept override;

protected:

    //! Participant Id
    types::ParticipantId id_;
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_VOIDPARTICIPANT_HPP_ */
