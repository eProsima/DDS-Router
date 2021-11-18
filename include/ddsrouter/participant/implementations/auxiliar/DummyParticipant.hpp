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

#include <ddsrouter/configuration/ParticipantConfiguration.hpp>
#include <ddsrouter/participant/implementations/auxiliar/EchoParticipant.hpp>
#include <ddsrouter/reader/implementations/auxiliar/DummyReader.hpp>
#include <ddsrouter/writer/implementations/auxiliar/DummyWriter.hpp>

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

    ~DummyParticipant();

    virtual ParticipantType type() const override;

    virtual std::shared_ptr<IWriter> create_writer(
            RealTopic topic) override;

    virtual std::shared_ptr<IReader> create_reader(
            RealTopic topic) override;

    virtual void add_discovered_endpoint(
            const Endpoint& new_endpoint);

    virtual Endpoint get_discovered_endpoint(
            const Guid& guid) const;

    void add_message_to_send(
            RealTopic topic,
            DataToSend data);

    std::vector<DataStored> data_received_ref(
            RealTopic topic);

    static DummyParticipant* get_participant(ParticipantId id);

protected:

    std::map<RealTopic, std::shared_ptr<DummyWriter>> writers_;

    std::map<RealTopic, std::shared_ptr<DummyReader>> readers_;

    static std::mutex static_mutex_;

    static std::map<ParticipantId, DummyParticipant*> participants_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DATABROKER_PARTICIPANT_IMPLEMENTATIONS_AUX_DUMMYPARTICIPANT_HPP_ */
