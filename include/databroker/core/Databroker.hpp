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
 * @file Databroker.hpp
 */

#ifndef _DATABROKER_CORE_DATABROKER_HPP_
#define _DATABROKER_CORE_DATABROKER_HPP_

#include <map>

#include <databroker/communication/Bridge.hpp>
#include <databroker/configuration/DatabrokerConfiguration.hpp>
#include <databroker/dynamic/AllowedTopicList.hpp>
#include <databroker/dynamic/DiscoveryDatabase.hpp>
#include <databroker/participant/IDatabrokerParticipant.hpp>
#include <databroker/participant/ParticipantDatabase.hpp>
#include <databroker/types/ReturnCode.hpp>
#include <databroker/types/ParticipantId.hpp>

namespace eprosima {
namespace databroker {

/**
 * TODO
 */
class Databroker
{
public:

    Databroker(DatabrokerConfiguration);

    ~Databroker();

    // EVENTS
    void reload_configuration(DatabrokerConfiguration);

    void endpoint_discovered(Endpoint);

protected:

    PayloadPool payload_pool_;

    std::shared_ptr<ParticipantDatabase> participants_;

    std::map<RealTopic, Bridge> bridges;

    DatabrokerConfiguration configuration_;

    AllowedTopicList allowed_topics_;

    std::shared_ptr<DiscoveryDatabase> discovery_database_;
};

} /* namespace databroker */
} /* namespace eprosima */

#endif /* _DATABROKER_CORE_DATABROKER_HPP_ */
