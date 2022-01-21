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
 * @file VoidParticipant.hpp
 */

#ifndef _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_AUX_VOIDPARTICIPANT_HPP_
#define _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_AUX_VOIDPARTICIPANT_HPP_

#include <ddsrouter/configuration/ParticipantConfiguration.hpp>
#include <ddsrouter/participant/IParticipant.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * Participant that has an empty implementation.
 * It does not discover anything.
 *
 * Writer: VoidWriter
 * Reader: VoidReader
 */
class VoidParticipant : public IParticipant
{
public:

    //! Constructor with Id of this participant
    VoidParticipant(
            const ParticipantId& id_);

    //! Override id() IParticipant method
    ParticipantId id() const noexcept override;

    //! Override type() IParticipant method
    ParticipantKind type() const noexcept override;

    //! Override create_writer() IParticipant method
    std::shared_ptr<IWriter> create_writer(
            RealTopic topic) override;

    //! Override create_reader() IParticipant method
    std::shared_ptr<IReader> create_reader(
            RealTopic topic) override;

    //! Override delete_writer() IParticipant method
    void delete_writer(
            std::shared_ptr<IWriter> writer) noexcept override;

    //! Override delete_reader() IParticipant method
    void delete_reader(
            std::shared_ptr<IReader> reader) noexcept override;

protected:

    //! Participant Id
    ParticipantId id_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_AUX_VOIDPARTICIPANT_HPP_ */
