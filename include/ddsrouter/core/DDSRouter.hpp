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
 * @file DDSRouter.hpp
 */

#ifndef _DDS_ROUTER_CORE_DDS_ROUTER_HPP_
#define _DDS_ROUTER_CORE_DDS_ROUTER_HPP_

#include <atomic>
#include <map>
#include <mutex>

#include <ddsrouter/communication/Bridge.hpp>
#include <ddsrouter/configuration/DDSRouterConfiguration.hpp>
#include <ddsrouter/dynamic/AllowedTopicList.hpp>
#include <ddsrouter/dynamic/DiscoveryDatabase.hpp>
#include <ddsrouter/participant/IDDSRouterParticipant.hpp>
#include <ddsrouter/participant/ParticipantDatabase.hpp>
#include <ddsrouter/participant/DDSRouterParticipantFactory.hpp>
#include <ddsrouter/types/ReturnCode.hpp>
#include <ddsrouter/types/ParticipantId.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * TODO
 */
class DDSRouter
{
public:

    DDSRouter(
            const DDSRouterConfiguration& configuration);

    virtual ~DDSRouter();

    // EVENTS
    void reload_configuration(
            const DDSRouterConfiguration& configuration);

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

    //! Active a topic within the DDSRouter context
    void active_topic_(
            const RealTopic& topic);

    //! Create a new bridge for a topic recently discovererd
    void create_new_bridge(
            const RealTopic& topic);

    //! Deactive a topic within the DDSRouter context
    void deactive_topic_(
            const RealTopic& topic);

    /////
    // DATA STORAGE

    std::shared_ptr<PayloadPool> payload_pool_;

    std::shared_ptr<ParticipantDatabase> participants_database_;

    std::shared_ptr<DiscoveryDatabase> discovery_database_;

    std::map<RealTopic, std::unique_ptr<Bridge>> bridges_;

    std::map<RealTopic, bool> current_topics_;

    DDSRouterConfiguration configuration_;

    AllowedTopicList allowed_topics_;

    DDSRouterParticipantFactory participant_factory_;

    /////
    // AUXILIAR VARIABLES

    //! Whether the DDSRouter has been initialized or stopped
    std::atomic<bool> enabled_;

    //! Internal mutex while initializing or closing
    std::recursive_mutex mutex_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDS_ROUTER_CORE_DDS_ROUTER_HPP_ */
