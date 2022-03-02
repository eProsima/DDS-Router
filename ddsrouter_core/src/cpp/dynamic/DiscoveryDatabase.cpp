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
 * @file DiscoveryDatabase.cpp
 *
 */

#include <ddsrouter_utils/exception/InconsistencyException.hpp>
#include <ddsrouter_utils/Log.hpp>

#include <dynamic/DiscoveryDatabase.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

using namespace eprosima::ddsrouter::core::types;

bool DiscoveryDatabase::topic_exists(
        const RealTopic& topic) const noexcept
{
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);

    // Loop over every Endpoint checking if topic matches
    // It is not required to be reference
    for (auto entity : entities_)
    {
        if (entity.second.topic() == topic)
        {
            return true;
        }
    }
    return false;
}

bool DiscoveryDatabase::endpoint_exists(
        const Guid& guid) const noexcept
{
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    return entities_.find(guid) != entities_.end();
}

bool DiscoveryDatabase::add_endpoint(
        const Endpoint& new_endpoint)
{
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);

    auto it = entities_.find(new_endpoint.guid());
    if (it != entities_.end())
    {
        // Already exists
        if (it->second.active())
        {
            throw utils::InconsistencyException(
                      utils::Formatter() <<
                          "Error adding Endpoint to database. Endpoint already exists." << new_endpoint);
        }
        else
        {
            // If exists but inactive, modify entry
            it->second = new_endpoint;

            logInfo(DDSROUTER_DISCOVERY_DATABASE,
                    "Modifying an already discovered (inactive) Endpoint " << new_endpoint << ".");

            return true;
        }
    }
    else
    {
        // Add it to the dictionary
        entities_.insert(std::pair<Guid, Endpoint>(new_endpoint.guid(), new_endpoint));

        logInfo(DDSROUTER_DISCOVERY_DATABASE, "Inserting a new discovered Endpoint " << new_endpoint << ".");

        return true;
    }
}

bool DiscoveryDatabase::update_endpoint(
        const Endpoint& new_endpoint)
{
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);

    auto it = entities_.find(new_endpoint.guid());
    if (it == entities_.end())
    {
        // Entry not found
        throw utils::InconsistencyException(
                  utils::Formatter() <<
                      "Error updating Endpoint in database. Endpoint entry not found." << new_endpoint);
    }
    else
    {
        // Modify entry
        it->second = new_endpoint;

        logInfo(DDSROUTER_DISCOVERY_DATABASE, "Modifying an already discovered Endpoint " << new_endpoint << ".");

        return true;
    }
}

utils::ReturnCode DiscoveryDatabase::erase_endpoint(
        const Guid& guid_of_endpoint_to_erase)
{
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);

    auto erased = entities_.erase(guid_of_endpoint_to_erase);

    if (erased == 0)
    {
        throw utils::InconsistencyException(
                  utils::Formatter() <<
                      "Error erasing Endpoint with GUID " << guid_of_endpoint_to_erase <<
                      " from database. Endpoint entry not found.");
    }
    else
    {
        return utils::ReturnCode::RETCODE_OK;
    }
}

Endpoint DiscoveryDatabase::get_endpoint(
        const Guid& endpoint_guid) const
{
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);

    auto it = entities_.find(endpoint_guid);
    if (it == entities_.end())
    {
        throw utils::InconsistencyException(
                  utils::Formatter() <<
                      "Error retrieving Endpoint with GUID " << endpoint_guid <<
                      " from database. Endpoint entry not found.");
    }

    return it->second;
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
