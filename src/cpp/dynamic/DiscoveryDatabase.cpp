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

#include <ddsrouter/dynamic/DiscoveryDatabase.hpp>
#include <ddsrouter/exceptions/UnsupportedException.hpp>

namespace eprosima {
namespace ddsrouter {

bool DiscoveryDatabase::topic_exists(
        const RealTopic& topic) const
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
        const Guid& guid) const
{
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    return entities_.find(guid) != entities_.end();
}

ReturnCode DiscoveryDatabase::add_or_modify_endpoint(
        const Endpoint& new_endpoint)
{
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);

    auto it = entities_.find(new_endpoint.guid());
    if (it != entities_.end())
    {
        // Already exists, modify it
        it->second = new_endpoint;
        return ReturnCode::RETCODE_OK;
    }
    else
    {
        // Add it to the dictionary
        entities_.insert(std::pair<Guid, Endpoint>(new_endpoint.guid(), new_endpoint));
        return ReturnCode::RETCODE_OK;
    }
}

ReturnCode DiscoveryDatabase::erase_endpoint(
        const Guid& guid_of_endpoint_to_erase)
{
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);

    auto erased = entities_.erase(guid_of_endpoint_to_erase);

    if (erased == 0)
    {
        return ReturnCode::RETCODE_NO_DATA;
    }
    else
    {
        return ReturnCode::RETCODE_OK;
    }
}

ReturnCode DiscoveryDatabase::erase_endpoint(
        const Endpoint& endpoint_to_erase)
{
    // Mutex is taken by erase_endpoint with guid, so it is not needed here
    return erase_endpoint(endpoint_to_erase.guid());
}

Endpoint DiscoveryDatabase::get_endpoint(
        const Guid& endpoint_guid) const
{
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);

    auto it = entities_.find(endpoint_guid);
    if (it == entities_.end())
    {
        // TODO: Add log warning
        return Endpoint();
    }

    return it->second;
}

} /* namespace ddsrouter */
} /* namespace eprosima */
