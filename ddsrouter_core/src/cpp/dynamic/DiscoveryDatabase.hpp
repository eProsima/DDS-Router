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
 * @file DiscoveryDatabase.hpp
 */

#ifndef __SRC_DDSROUTERCORE_DYNAMIC_DISCOVERYDATABASE_HPP_
#define __SRC_DDSROUTERCORE_DYNAMIC_DISCOVERYDATABASE_HPP_

#include <map>
#include <shared_mutex>
#include <string>

#include <ddsrouter_core/types/endpoint/Endpoint.hpp>
#include <ddsrouter_core/types/dds/Guid.hpp>
#include <ddsrouter_utils/ReturnCode.hpp>
#include <ddsrouter_core/types/topic/RealTopic.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Class that stores a collection of discovered remote (not belonging to this DDSRouter) Endpoints.
 */
class DiscoveryDatabase
{
public:

    /**
     * @brief Whether a topic exists in any Endpoint in the database
     *
     * @param [in] topic: topic to check if it exists
     * @return true if any endpoint has this topic, false otherwise
     */
    bool topic_exists(
            const types::RealTopic& topic) const noexcept;

    //! Whether this guid is in the database
    bool endpoint_exists(
            const types::Guid& guid) const noexcept;

    /**
     * @brief Add a new endpoint to the database.
     *
     * @param [in] new_endpoint: new endpoint to store
     * @return true if the endpoint has been added
     * @throw \c InconsistencyException in case an endpoint with the same guid already exists and is active
     */
    bool add_endpoint(
            const types::Endpoint& new_endpoint);

    /**
     * @brief Update an entry of the database by replacing the stored endpoint by a new one.
     *
     * @param [in] new_endpoint: new endpoint to store
     * @return true if the endpoint has been updated
     * @throw \c InconsistencyException in case there is no entry associated to this endpoint
     */
    bool update_endpoint(
            const types::Endpoint& new_endpoint);

    /**
     * @brief Erase an endpoint inside the database
     *
     * @param [in] guid_of_endpoint_to_erase guid of endpoint that will be erased
     * @return \c RETCODE_OK if correctly erased
     * @throw \c InconsistencyException in case there is no entry associated to this guid
     */
    utils::ReturnCode erase_endpoint(
            const types::Guid& guid_of_endpoint_to_erase);

    /**
     * @brief Get the endpoint object with this guid
     *
     * @param [in] guid: guid to query
     * @return Endpoint referring to this guid
     * @throw \c InconsistencyException in case there is no entry associated to this guid
     */
    types::Endpoint get_endpoint(
            const types::Guid& endpoint_guid) const;

    // TODO
    // some way of allowing participants to subscribe to a callback when new information arrives

protected:

    //! Database of endpoints indexed by guid
    std::map<types::Guid, types::Endpoint> entities_;

    //! Mutex to guard queries to the database
    mutable std::shared_timed_mutex mutex_;
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_DYNAMIC_DISCOVERYDATABASE_HPP_ */
