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
 * TODO
 */
class VoidParticipant : public IParticipant
{
public:

    VoidParticipant(
            ParticipantConfiguration participant_configuration);

    virtual ~VoidParticipant();

    ParticipantId id() const override;

    virtual ParticipantType type() const override;

    virtual std::shared_ptr<IWriter> create_writer(
            RealTopic) override;

    virtual std::shared_ptr<IReader> create_reader(
            RealTopic) override;

    virtual void delete_writer(
            std::shared_ptr<IWriter> writer) override;

    virtual void delete_reader(
            std::shared_ptr<IReader> reader) override;

protected:

    ParticipantConfiguration configuration_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_AUX_VOIDPARTICIPANT_HPP_ */
