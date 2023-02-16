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

#include <ddspipe_participants/library/library_dll.h>

namespace eprosima {
namespace ddspipe {
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
    DDSPIPE_PARTICIPANTS_DllAPI BlankParticipant(
            const core::types::ParticipantId& id_);

    //! Override id() IParticipant method
    DDSPIPE_PARTICIPANTS_DllAPI core::types::ParticipantId id() const noexcept override;

    //! Override is_repeater() IParticipant method
    DDSPIPE_PARTICIPANTS_DllAPI bool is_repeater() const noexcept override;

    //! Override is_rtps_kind() IParticipant method
    DDSPIPE_PARTICIPANTS_DllAPI bool is_rtps_kind() const noexcept override;

    //! Override create_writer() IParticipant method
    DDSPIPE_PARTICIPANTS_DllAPI std::shared_ptr<core::IWriter> create_writer(
            const core::ITopic& topic) override;

    //! Override create_reader() IParticipant method
    DDSPIPE_PARTICIPANTS_DllAPI std::shared_ptr<core::IReader> create_reader(
            const core::ITopic& topic) override;

protected:

    //! Participant Id
    core::types::ParticipantId id_;
};

} /* namespace participants */
} /* namespace ddspipe */
} /* namespace eprosima */
