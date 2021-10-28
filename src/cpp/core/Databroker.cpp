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

#include <databroker/configuration/DatabrokerConfiguration.hpp>
#include <databroker/core/Databroker.hpp>

namespace eprosima {
namespace databroker {

// TODO: Add logs

Databroker::Databroker(const DatabrokerConfiguration& configuration)
        : payload_pool_(new PayloadPool())
        , participants_database_(new ParticipantDatabase())
        , discovery_database_(new DiscoveryDatabase())
        , allowed_topics_()
        , bridges()
        , configuration_(configuration)
        , participant_factory_()
        , enabled_(false)
{
    // TODO
}

Databroker::~Databroker()
{
    // TODO
    // There is no need to destroy shared ptrs as they will delete itslefs with 0 references
}

ReturnCode Databroker::init()
{
    // Init own congiguration
    configuration_.load();
    // Init topic allowed
    init_allowes_topics_();
    // Load Participants
    init_participants_();
    // Create Bridges
    init_bridges_();

    return ReturnCode::RETCODE_OK;
}

// EVENTS
void Databroker::stop()
{
    // TODO
}

void Databroker::reload_configuration(const DatabrokerConfiguration&)
{
    // TODO
}

void Databroker::endpoint_discovered(const Endpoint&)
{
    // TODO
}

ReturnCode Databroker::init_allowes_topics_()
{
    // TODO
}

ReturnCode Databroker::init_participants_()
{
    // TODO
}

ReturnCode Databroker::init_bridges_()
{
    // TODO
}

} /* namespace databroker */
} /* namespace eprosima */
