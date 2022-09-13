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
    , enabled_(false)
{
    logDebug(DDSROUTER_DISCOVERY_DATABASE, "Creating queue processing thread.");
}

DiscoveryDatabase::~DiscoveryDatabase()
{
    logDebug(DDSROUTER_DISCOVERY_DATABASE, "Destroying Discovery Database.");

    disable();
}

void DiscoveryDatabase::enable() noexcept
{
    if (!enabled_.load())
    {
        queue_processing_thread_ = std::thread(&DiscoveryDatabase::queue_processing_thread_routine_, this);
        enabled_.store(true);
        logDebug(DDSROUTER_DISCOVERY_DATABASE, "Creating queue processing thread routine.");
    }
    else
    {
        logDebug(DDSROUTER_DISCOVERY_DATABASE, "Processing thread routine already enabled.");
    }
}

void DiscoveryDatabase::disable() noexcept
{
    if (enabled_.load())
    {
        clear_all_callbacks();

        {
            std::lock_guard<std::mutex> lock(entities_to_process_cv_mutex_);
            exit_.store(true);
        }
        entities_to_process_cv_.notify_one();

        logDebug(DDSROUTER_DISCOVERY_DATABASE, "Waiting for queue processing thread to finish.");
        enabled_.store(false);
        queue_processing_thread_.join();
    }
    else
    {
        logDebug(DDSROUTER_DISCOVERY_DATABASE, "Processing thread routine already stopped.");
    }
}

bool DiscoveryDatabase::topic_exists(
        const RealTopic& topic) const noexcept
{
    logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling topic_exists BEFORE");
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling topic_exists AFTER");

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
    logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling endpoint_exists BEFORE");
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling endoint_exists AFTER");
    return entities_.find(guid) != entities_.end();
}

bool DiscoveryDatabase::add_endpoint_(
        const Endpoint& new_endpoint)
{
    {
    logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling add_endpoint_ BEFORE");
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling add_endpoint_ AFTER");

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
        logInfo(DDSROUTER_DISCOVERY_DATABASE, "Inserting a new discovered Endpoint " << new_endpoint << ".");

        // Add it to the dictionary
        entities_.insert(std::pair<Guid, Endpoint>(new_endpoint.guid(), new_endpoint));
    }
    }

    {
        std::lock_guard<std::mutex> lock(callbacks_mutex_);
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
    {
    logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling update_endpoint_ BEFORE");
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling update_endpoint_ AFTER");

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

    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    for (auto updated_endpoint_callback : updated_endpoint_callbacks_)
    {
        updated_endpoint_callback(endpoint_to_update);
    }

    return true;
}

utils::ReturnCode DiscoveryDatabase::erase_endpoint_(
        const Endpoint& endpoint_to_erase)
{
    {
    logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling erase_endpoint_ BEFORE");
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling erase_endpoint_ AFTER");

    auto erased = entities_.erase(endpoint_to_erase.guid());

    logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling erase_endpoint_ AFTER ERASING");

    if (erased == 0)
    {
        throw utils::InconsistencyException(
                  utils::Formatter() <<
                      "Error erasing Endpoint " << endpoint_to_erase <<
                      " from database. Endpoint entry not found.");
    }
    }

    std::lock_guard<std::mutex> lock(callbacks_mutex_);

    logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling erase_endpoint_ BEFORE CALLBACKS");
    for (auto erased_endpoint_callback : erased_endpoint_callbacks_)
    {
        logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling erase_endpoint_ CALLING CALLBACK");
        erased_endpoint_callback(endpoint_to_erase);
    }
    logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling erase_endpoint_ AFTER CALLBACKS");

    return utils::ReturnCode::RETCODE_OK;
}

void DiscoveryDatabase::add_endpoint(
        const Endpoint& new_endpoint) noexcept
{
    logDebug(DDSROUTER_DISCOVERY_DATABASE, "Adding endpoint");
    push_item_to_queue_(std::make_tuple(DatabaseOperation::add, new_endpoint));
}

void DiscoveryDatabase::update_endpoint(
        const Endpoint& endpoint_to_update) noexcept
{
    logDebug(DDSROUTER_DISCOVERY_DATABASE, "Updating endpoint");
    push_item_to_queue_(std::make_tuple(DatabaseOperation::update, endpoint_to_update));
}

void DiscoveryDatabase::erase_endpoint(
        const Endpoint& endpoint_to_erase) noexcept
{
    logDebug(DDSROUTER_DISCOVERY_DATABASE, "Erasing endpoint");
    push_item_to_queue_(std::make_tuple(DatabaseOperation::erase, endpoint_to_erase));
}

Endpoint DiscoveryDatabase::get_endpoint(
        const Guid& endpoint_guid) const
{
    logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling get_endpoint BEFORE");
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling get_endpoint AFTER");

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
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    added_endpoint_callbacks_.push_back(endpoint_discovered_callback);
}

void DiscoveryDatabase::add_endpoint_updated_callback(
        std::function<void(Endpoint)> endpoint_updated_callback) noexcept
{
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    updated_endpoint_callbacks_.push_back(endpoint_updated_callback);
}

void DiscoveryDatabase::add_endpoint_erased_callback(
        std::function<void(Endpoint)> endpoint_erased_callback) noexcept
{
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    erased_endpoint_callbacks_.push_back(endpoint_erased_callback);
}

void DiscoveryDatabase::clear_all_callbacks() noexcept
{
    logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling clear_all_callbacks BEFORE");
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling clear_all_callbacks AFTER");

    added_endpoint_callbacks_.clear();
    updated_endpoint_callbacks_.clear();
    erased_endpoint_callbacks_.clear();
}

void DiscoveryDatabase::queue_processing_thread_routine_() noexcept
{
    while (true)
    {
        {
            logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling queue_processing_thread_routine_ BEFORE");
            std::unique_lock<std::mutex> lock(entities_to_process_cv_mutex_);
            logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling queue_processing_thread_routine_ AFTER");
            entities_to_process_cv_.wait(
                lock,
                [&]
                {
                    return !entities_to_process_.BothEmpty() || exit_.load();
                });
            logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling queue_processing_thread_routine_ OUT OF WAIT");
            if (exit_.load())
            {
                break;
            }
        }
        // Release the mutex as DBQueue already has internal mutexes
        logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling process_queue_ BEFORE");
        process_queue_();
        logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling process_queue_ AFTER");
    }
}

void DiscoveryDatabase::push_item_to_queue_(
        std::tuple<DatabaseOperation, Endpoint> item) noexcept
{
    {
        logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling push_item_to_queue_ BEFORE");
        std::lock_guard<std::mutex> lock(entities_to_process_cv_mutex_);
        logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling push_item_to_queue_ AFTER");
        entities_to_process_.Push(item);
        logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling push_item_to_queue_ AFTER PUSHING");
    }
    entities_to_process_cv_.notify_one();
}

void DiscoveryDatabase::process_queue_() noexcept
{
    logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling process_queue_ BEFORE SWAP");
    entities_to_process_.Swap();
    logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling process_queue_ AFTER SWAP");
    while (!entities_to_process_.Empty())
    {
        logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling process_queue_ NEW ITERATION");
        std::tuple<DatabaseOperation, Endpoint> queue_item = entities_to_process_.Front();
        DatabaseOperation db_operation = std::get<0>(queue_item);
        Endpoint entity = std::get<1>(queue_item);
        try
        {
            if (db_operation == DatabaseOperation::add)
            {
                add_endpoint_(entity);
            }
            else if (db_operation == DatabaseOperation::update)
            {
                update_endpoint_(entity);
            }
            else if (db_operation == DatabaseOperation::erase)
            {
                erase_endpoint_(entity);
            }
        }
        catch (const utils::InconsistencyException& e)
        {
            logDevError(DDSROUTER_DISCOVERY_DATABASE,
                    "Error processing database operations queue:" << e.what() << ".");
        }
        logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling process_queue_ BEFORE POP");
        entities_to_process_.Pop();
        logDebug(DDSROUTER_DISCOVERY_DATABASE, "Calling process_queue_ AFTER POP");
    }
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */
