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
 * @file DatabrokerSharedDatabase.hpp
 */

#ifndef _DATABROKER_CORE_DATABROKERSHAREDDATABASE_HPP_
#define _DATABROKER_CORE_DATABROKERSHAREDDATABASE_HPP_

#include <databroker/dynamic/DiscoveryDatabase.hpp>
#include <databroker/participant/ParticipantDatabase.hpp>
#include <databroker/types/ReturnCode.hpp>

namespace eprosima {
namespace databroker {

/**
 * TODO
 */
class DatabrokerSharedDatabase
{
public:

    DatabrokerSharedDatabase(const DatabrokerConfiguration& configuration);

    virtual ~DatabrokerSharedDatabase();

    ReturnCode init();

    // EVENTS

    void stop();

    void reload_configuration(const DatabrokerConfiguration& configuration);

    void endpoint_discovered(const Endpoint& endpoint);

protected:

    /////
    // INTERNAL METHODS

    //! Load allowed topics from configuration
    ReturnCode init_allowes_topics_();

    //! Load participants to the participant database from configuration
    ReturnCode init_participants_();

    //! Create bridges from topics
    ReturnCode init_bridges_();

    /////
    // DATA STORAGE

    std::shared_ptr<PayloadPool> payload_pool_;

    std::shared_ptr<ParticipantDatabase> participants_database_;

    std::shared_ptr<DiscoveryDatabase> discovery_database_;

    std::map<RealTopic, Bridge> bridges;

    DatabrokerConfiguration configuration_;

    AllowedTopicList allowed_topics_;

    DatabrokerParticipantFactory participant_factory_;

    /////
    // AUXILIAR VARIABLES

    //! Whether the DatabrokerSharedDatabase has been initialized or stopped
    bool enabled_;

};

} /* namespace databroker */
} /* namespace eprosima */

#endif /* _DATABROKER_CORE_DATABROKERSHAREDDATABASE_HPP_ */
