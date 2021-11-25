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

#ifndef _DDSROUTER_DYNAMIC_DISCOVERYDATABASE_HPP_
#define _DDSROUTER_DYNAMIC_DISCOVERYDATABASE_HPP_

#include <map>
#include <shared_mutex>
#include <string>

#include <ddsrouter/types/endpoint/Endpoint.hpp>
#include <ddsrouter/types/endpoint/Guid.hpp>
#include <ddsrouter/types/ReturnCode.hpp>
#include <ddsrouter/types/topic/RealTopic.hpp>

namespace eprosima {
namespace ddsrouter {

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
            const RealTopic& topic) const noexcept;

    //! Whether this guid is in the database
    bool endpoint_exists(
            const Guid& guid) const noexcept;

    /**
     * @brief Add a new endpoint or modify it inside the database in case it already exists.
     *
     * @param [in] new_endpoint: new endpoint to store
     * @return true if the endpoint has been updated, false if it has been added
     */
    bool add_or_modify_endpoint(
            const Endpoint& new_endpoint) noexcept;

    /**
     * @brief Erase an endpoint inside the database
     *
     * @param [in] guid_of_endpoint_to_erase guid of endpoint that will be erased
     * @return \c RETCODE_OK if correctly erased
     * @return \c RETCODE_NO_DATA if the endpoint was not in the database
     */
    ReturnCode erase_endpoint(
            const Guid& guid_of_endpoint_to_erase) noexcept;

    /**
     * @brief Get the endpoint object with this guid
     *
     * @param [in] guid: guid to query
     * @return Endpoint referring to this guid. In case the endpoint does not exist, return an invalid endpoint.
     */
    Endpoint get_endpoint(
            const Guid& endpoint_guid) const noexcept;

    // TODO
    // some way of allowing participants to subscribe to a callback when new information arrives

protected:

    //! Database of endpoints indexed by guid
    std::map<Guid, Endpoint> entities_;

    //! Mutex to guard queries to the database
    mutable std::shared_timed_mutex mutex_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_DYNAMIC_DISCOVERYDATABASE_HPP_ */
