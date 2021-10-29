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

#include <atomic>
#include <map>
#include <mutex>

#include <databroker/communication/Bridge.hpp>
#include <databroker/configuration/DatabrokerConfiguration.hpp>
#include <databroker/dynamic/AllowedTopicList.hpp>
#include <databroker/dynamic/DiscoveryDatabase.hpp>
#include <databroker/participant/IDatabrokerParticipant.hpp>
#include <databroker/participant/ParticipantDatabase.hpp>
#include <databroker/participant/DatabrokerParticipantFactory.hpp>
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

    Databroker(
            const DatabrokerConfiguration& configuration);

    virtual ~Databroker();

    // EVENTS
    void reload_configuration(
            const DatabrokerConfiguration& configuration);

    void endpoint_discovered(
            const Endpoint& endpoint);

protected:

    /////
    // INTERNAL METHODS

    //! Load allowed topics from configuration
    void init_allowed_topics_();

    //! Create participants and add it to the participants database
    void init_participants_();

    //! Create bridges from topics
    void init_bridges_();

    //! New Topic found, check if it shuld be activated
    void discovered_topic_(
            const RealTopic& topic);

    //! Active a topic within the Databroker context
    void active_topic_(
            const RealTopic& topic);

    //! Create a new bridge for a topic recently discovererd
    void create_new_bridge(
            const RealTopic& topic);

    //! Deactive a topic within the Databroker context
    void deactive_topic_(
            const RealTopic& topic);

    /////
    // DATA STORAGE

    std::shared_ptr<PayloadPool> payload_pool_;

    std::shared_ptr<ParticipantDatabase> participants_database_;

    std::shared_ptr<DiscoveryDatabase> discovery_database_;

    std::map<RealTopic, Bridge> bridges_;

    std::map<RealTopic, bool> current_topics_;

    DatabrokerConfiguration configuration_;

    AllowedTopicList allowed_topics_;

    DatabrokerParticipantFactory participant_factory_;

    /////
    // AUXILIAR VARIABLES

    //! Whether the Databroker has been initialized or stopped
    std::atomic<bool> enabled_;

    //! Internal mutex while initializing or closing
    std::recursive_mutex mutex_;
};

} /* namespace databroker */
} /* namespace eprosima */

#endif /* _DATABROKER_CORE_DATABROKER_HPP_ */
