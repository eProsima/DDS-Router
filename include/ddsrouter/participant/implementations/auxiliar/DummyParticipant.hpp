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

#ifndef _DATABROKER_PARTICIPANT_IMPLEMENTATIONS_AUX_DUMMYPARTICIPANT_HPP_
#define _DATABROKER_PARTICIPANT_IMPLEMENTATIONS_AUX_DUMMYPARTICIPANT_HPP_

#include <ddsrouter/participant/implementations/auxiliar/EchoParticipant.hpp>
#include <ddsrouter/configuration/ParticipantConfiguration.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * TODO
 */
class DummyParticipant : public EchoParticipant
{
public:

    DummyParticipant(
        ParticipantConfiguration participant_configuration,
        std::shared_ptr<DiscoveryDatabase> discovery_database);

    virtual ParticipantType type() const override;

    virtual std::shared_ptr<IWriter> create_writer(
            RealTopic) override;

    virtual std::shared_ptr<IReader> create_reader(
            RealTopic) override;

    virtual void add_discovered_endpoint(const Endpoint& new_endpoint);

    virtual Endpoint get_discovered_endpoint(const Guid& guid) const;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DATABROKER_PARTICIPANT_IMPLEMENTATIONS_AUX_DUMMYPARTICIPANT_HPP_ */
