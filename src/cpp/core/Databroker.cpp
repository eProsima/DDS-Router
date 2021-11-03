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
 * @file DataBroker.cpp
 *
 */

#include <cassert>

#include <databroker/configuration/DatabrokerConfiguration.hpp>
#include <databroker/core/Databroker.hpp>
#include <databroker/exceptions/UnsupportedException.hpp>
#include <databroker/exceptions/InitializationException.hpp>

namespace eprosima {
namespace databroker {

// TODO: Add logs

Databroker::Databroker(
        const DatabrokerConfiguration& configuration)
    : payload_pool_(new PayloadPool())
    , participants_database_(new ParticipantDatabase())
    , discovery_database_(new DiscoveryDatabase())
    , allowed_topics_()
    , bridges_()
    , configuration_(configuration)
    , participant_factory_()
    , enabled_(false)
{
    // Init topic allowed
    init_allowed_topics_();
    // Load Participants
    init_participants_();
    // Create Bridges
    init_bridges_();

    enabled_.store(true);
}

Databroker::~Databroker()
{
    // TODO
    // There is no need to destroy shared ptrs as they will delete itslefs with 0 references
}

void Databroker::reload_configuration(
        const DatabrokerConfiguration&)
{
    // TODO
    throw UnsupportedException("Databroker::reload_configuration not supported yet");
}

void Databroker::endpoint_discovered(
        const Endpoint&)
{
    // TODO
    throw UnsupportedException("Databroker::endpoint_discovered not supported yet");
}

void Databroker::init_allowed_topics_()
{
    allowed_topics_.reload(
        configuration_.whitelist(),
        configuration_.blacklist());
}

void Databroker::init_participants_()
{
    for (std::pair<const eprosima::databroker::ParticipantId, eprosima::databroker::RawConfiguration> participant_info :
            configuration_.participants_configurations())
    {
        std::shared_ptr<IDatabrokerParticipant> new_participant;

        // Create participant
        // This should not be in try catch case as if it fails the whole init must fail
        new_participant =
                participant_factory_.create_participant(
            participant_info.first,
            participant_info.second,
            payload_pool_,
            discovery_database_);

        // create_participant should throw an exception in fail, never return nullptr
        assert(nullptr != new_participant);

        participants_database_->add_participant(
            participant_info.first,
            new_participant);
    }
}

void Databroker::init_bridges_()
{
    for (RealTopic topic : configuration_.real_topics())
    {
        discovered_topic_(topic);
    }
}

void Databroker::discovered_topic_(
        const RealTopic& topic)
{
    if (allowed_topics_.is_topic_allowed(topic))
    {
        active_topic_(topic);
    }
}

void Databroker::active_topic_(
        const RealTopic& topic)
{
    auto it_topic = current_topics_.find(topic);

    if (it_topic == current_topics_.end())
    {
        // The topic does not exist
        current_topics_[topic] = true;

        create_new_bridge(topic);
    }
    else
    {
        // The topic already exists, so activate it. Bridge handles double activation
        auto it_bridge = bridges_.find(topic);

        // The bridges and the current topics must be coherent
        assert(it_bridge != bridges_.end());

        it_bridge->second.enable();
    }
}

void Databroker::create_new_bridge(
        const RealTopic& topic)
{
    bridges_.emplace(topic, Bridge(topic, participants_database_));
}

void Databroker::deactive_topic_(
        const RealTopic&)
{
    // TODO
    throw UnsupportedException("Databroker::reload_configuration not supported yet");
}

} /* namespace databroker */
} /* namespace eprosima */
