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

DiscoveryDatabase::DiscoveryDatabase() noexcept
    : exit_(false)
{
    logDebug(DDSROUTER_DISCOVERY_DATABASE, "Creating queue processing thread.");
    queue_processing_thread_ = std::thread(&DiscoveryDatabase::queue_processing_thread_routine_, this);
}

DiscoveryDatabase::~DiscoveryDatabase()
{
    logDebug(DDSROUTER_DISCOVERY_DATABASE, "Destroying Discovery Database.");
    {
        std::lock_guard<std::mutex> lock(entities_to_process_cv_mutex_);
        exit_.store(true);
    }
    entities_to_process_cv_.notify_one();

    logDebug(DDSROUTER_DISCOVERY_DATABASE, "Waiting for queue processing thread to finish.");
    queue_processing_thread_.join();
}

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

bool DiscoveryDatabase::add_endpoint_(
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

        for (auto added_endpoint_callback : added_endpoint_callbacks_)
        {
            added_endpoint_callback(new_endpoint);
        }

        return true;
    }
}

bool DiscoveryDatabase::update_endpoint_(
        const Endpoint& endpoint_to_update)
{
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);

    auto it = entities_.find(endpoint_to_update.guid());
    if (it == entities_.end())
    {
        // Entry not found
        throw utils::InconsistencyException(
                  utils::Formatter() <<
                      "Error updating Endpoint in database. Endpoint entry not found." << endpoint_to_update);
    }
    else
    {
        // Modify entry
        it->second = endpoint_to_update;
        // It is assumed a topic cannot change, otherwise further actions may be taken

        logInfo(DDSROUTER_DISCOVERY_DATABASE, "Modifying an already discovered Endpoint " << endpoint_to_update << ".");

        for (auto updated_endpoint_callback : updated_endpoint_callbacks_)
        {
            updated_endpoint_callback(endpoint_to_update);
        }

        return true;
    }
}

utils::ReturnCode DiscoveryDatabase::erase_endpoint_(
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
        Endpoint erased_endpoint = get_endpoint(guid_of_endpoint_to_erase);
        for (auto erased_endpoint_callback : erased_endpoint_callbacks_)
        {
            erased_endpoint_callback(erased_endpoint);
        }

        return utils::ReturnCode::RETCODE_OK;
    }
}

void DiscoveryDatabase::add_endpoint(
        const Endpoint& new_endpoint) noexcept
{
    push_item_to_queue_(std::make_tuple(DatabaseOperation::ADD, new_endpoint));
}

void DiscoveryDatabase::update_endpoint(
        const Endpoint& endpoint_to_update) noexcept
{
    push_item_to_queue_(std::make_tuple(DatabaseOperation::UPDATE, endpoint_to_update));
}

void DiscoveryDatabase::erase_endpoint(
        const Endpoint& endpoint_to_erase) noexcept
{
    push_item_to_queue_(std::make_tuple(DatabaseOperation::ERASE, endpoint_to_erase));
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

void DiscoveryDatabase::add_endpoint_discovered_callback(
        std::function<void(Endpoint)> endpoint_discovered_callback) noexcept
{
    added_endpoint_callbacks_.push_back(endpoint_discovered_callback);
}

void DiscoveryDatabase::queue_processing_thread_routine_() noexcept
{
    while (true)
    {
        {
            std::unique_lock<std::mutex> lock(entities_to_process_cv_mutex_);
            entities_to_process_cv_.wait(
                lock,
                [&]
                {
                    return !entities_to_process_.BothEmpty() || exit_.load();
                });

            if (exit_.load())
            {
                break;
            }
        }
        // Release the mutex as DBQueue already has internal mutexes
        process_queue_();
    }
}

void DiscoveryDatabase::push_item_to_queue_(
        std::tuple<DatabaseOperation, Endpoint> item) noexcept
{
    {
        std::lock_guard<std::mutex> lock(entities_to_process_cv_mutex_);
        entities_to_process_.Push(item);
    }
    entities_to_process_cv_.notify_one();
}

void DiscoveryDatabase::process_queue_() noexcept
{
    entities_to_process_.Swap();
    while (!entities_to_process_.Empty())
    {
        std::tuple<DatabaseOperation, Endpoint> queue_item = entities_to_process_.Front();
        DatabaseOperation db_operation = std::get<0>(queue_item);
        Endpoint entity = std::get<1>(queue_item);
        try
        {
            if (db_operation == DatabaseOperation::ADD)
            {
                add_endpoint_(entity);
            }
            else if (db_operation == DatabaseOperation::UPDATE)
            {
                update_endpoint_(entity);
            }
            else if (db_operation == DatabaseOperation::ERASE)
            {
                erase_endpoint_(entity.guid());
            }
        }
        catch (const utils::InconsistencyException& e)
        {
            logError(DDSROUTER_DISCOVERY_DATABASE,
                    "Error processing database operations queue:" << e.what() << ".");
        }
        entities_to_process_.Pop();
    }
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
