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
 * @file BlankParticipant.hpp
 */

#ifndef __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_BLANKPARTICIPANT_HPP_
#define __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_BLANKPARTICIPANT_HPP_

#include <ddsrouter_core/participants/participant/configuration/ParticipantConfiguration.hpp>
#include <ddsrouter_core/participant/IParticipant.hpp>

namespace eprosima {
namespace ddsrouter {
namespace participants {

/**
 * Participant that has an empty implementation.
 * It does not discover anything.
 *
 * Writer: BlankWriter
 * Reader: BlankReader
 */
class BlankParticipant : public core::IParticipant
{
public:

    //! Constructor with Id of this participant
    BlankParticipant(
            const core::types::ParticipantId& id_);

    //! Override id() IParticipant method
    core::types::ParticipantId id() const noexcept override;

    //! Override is_repeater() IParticipant method
    bool is_repeater() const noexcept override;

    //! Override is_rtps_kind() IParticipant method
    bool is_rtps_kind() const noexcept override;

    //! Override create_writer() IParticipant method
    std::shared_ptr<core::IWriter> create_writer(
            core::types::DdsTopic topic) override;

    //! Override create_reader() IParticipant method
    std::shared_ptr<core::IReader> create_reader(
            core::types::DdsTopic topic) override;

    //! Override delete_writer() IParticipant method
    void delete_writer(
            std::shared_ptr<core::IWriter> writer) noexcept override;

    //! Override delete_reader() IParticipant method
    void delete_reader(
            std::shared_ptr<core::IReader> reader) noexcept override;

    virtual void start() override;

protected:

    //! Participant Id
    core::types::ParticipantId id_;
};

} /* namespace participants */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_BLANKPARTICIPANT_HPP_ */
