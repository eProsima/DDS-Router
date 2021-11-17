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

#include <ddsrouter/participant/IParticipant.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * TODO
 */
class VoidParticipant : public IParticipant
{
public:

    VoidParticipant(
            ParticipantId id);

    virtual ~VoidParticipant();

    ParticipantId id() const override;

    ParticipantType type() const override;

    std::shared_ptr<IWriter> create_writer(
            RealTopic) override;

    std::shared_ptr<IReader> create_reader(
            RealTopic) override;

protected:

    ParticipantId id_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_AUX_VOIDPARTICIPANT_HPP_ */
