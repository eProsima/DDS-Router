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
 * @file SingletonDummyParticipant.hpp
 */

#ifndef _DATABROKER_PARTICIPANT_IMPLEMENTATIONS_AUX_SINGLETONDUMMYPARTICIPANT_HPP_
#define _DATABROKER_PARTICIPANT_IMPLEMENTATIONS_AUX_SINGLETONDUMMYPARTICIPANT_HPP_

#include <ddsrouter/participant/IParticipant.hpp>
#include <ddsrouter/participant/implementations/auxiliar/DummyParticipant.hpp>
#include <ddsrouter/configuration/ParticipantConfiguration.hpp>
#include <ddsrouter/reader/implementations/auxiliar/DummyReader.hpp>
#include <ddsrouter/writer/implementations/auxiliar/DummyWriter.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * TODO
 */
class SingletonDummyParticipant : public IParticipant
{
public:

    /**
     * Returns the SingletonDummyParticipant singleton.
     * @return The SingletonDummyParticipant singleton.
     */
    static std::shared_ptr<SingletonDummyParticipant> get_instance();

    ParticipantId id() const override;

    ParticipantType type() const override;

    std::shared_ptr<IWriter> create_writer(
            RealTopic) override;

    std::shared_ptr<IReader> create_reader(
            RealTopic) override;

    /////
    // Specific methods to interact with writers and readers
    void add_discovered_endpoint(const Endpoint& new_endpoint);

    Endpoint get_discovered_endpoint(const Guid& guid) const;

    void add_message_to_send(RealTopic topic, DataToSend data);

    std::vector<DataStoraged>& data_received_ref(RealTopic topic);

    /////
    // Methods to set configuration
    void set_configuration(ParticipantConfiguration participant_configuration);

    void set_discovery_database(std::shared_ptr<DiscoveryDatabase> discovery_database);

protected:

    SingletonDummyParticipant();

    ParticipantConfiguration participant_configuration_;

    std::shared_ptr<DiscoveryDatabase> discovery_database_;

    std::map<RealTopic, std::shared_ptr<DummyWriter>> writers_;

    std::map<RealTopic, std::shared_ptr<DummyReader>> readers_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DATABROKER_PARTICIPANT_IMPLEMENTATIONS_AUX_SINGLETONDUMMYPARTICIPANT_HPP_ */
