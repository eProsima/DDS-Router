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

#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <map>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>

#include <fastrtps/utils/DBQueue.h>

#include <ddspipe_core/types/dds/Endpoint.hpp>
#include <ddspipe_core/types/dds/Guid.hpp>
#include <cpp_utils/ReturnCode.hpp>
#include <ddspipe_core/types/topic/dds/DistributedTopic.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {

//! Operations to perform on a DiscoveryDatabase
DDSPIPE_CORE_DllAPI enum class DatabaseOperation
{
    add,
    update,
    erase
};

/**
 * Class that stores a collection of discovered remote (not belonging to this DDSRouter) Endpoints.
 */
class DiscoveryDatabase
{
public:

    /**
     * @brief Construct a new DiscoveryDatabase object
     *
     * Create a thread in charge of processing DB transactions stored in a queue.
     * Call \c start() function to enable the DiscoveryDatabase.
     */
    DDSPIPE_CORE_DllAPI DiscoveryDatabase() noexcept;

    /**
     * @brief Destroy the DiscoveryDatabase object
     *
     * Signal and wait for the queue processing thread to stop.
     */
    DDSPIPE_CORE_DllAPI ~DiscoveryDatabase();

    /**
     * @brief Initialize the queue processing thread routine
     *
     * Create a thread in charge of processing the dynamic discovery of endpoints.
     * The creation of the discovery database processing thread is done after the creation of the DiscoveryDatabase due
     * to a data race condition that may involve first creating the topics through the discovered endpoints or creating
     * them from the builtin topics configuration.
     * The builtin topics may have special topic configurations not detected in discovery
     * that would mean that the topic does not have the correct configuration.
     */
    DDSPIPE_CORE_DllAPI void start() noexcept;

    /**
     * @brief Stop the queue processing thread routine
     *
     * Join the thread in charge of processing the dynamic discovery of endpoints.
     */
    DDSPIPE_CORE_DllAPI void stop() noexcept;

    /**
     * @brief Whether a topic exists in any Endpoint in the database
     *
     * @param [in] topic: topic to check if it exists
     * @return true if any endpoint has this topic, false otherwise
     */
    DDSPIPE_CORE_DllAPI bool topic_exists(
            const types::DdsTopic& topic) const noexcept;

    //! Whether this guid is in the database
    DDSPIPE_CORE_DllAPI bool endpoint_exists(
            const types::Guid& guid) const noexcept;

    /**
     * @brief Insert endpoint to the database
     *
     * This method stores an insert operation in an internal queue, being this operation then performed by a
     * dedicated thread.
     *
     * @param [in] new_endpoint: new endpoint to store
     */
    DDSPIPE_CORE_DllAPI void add_endpoint(
            const types::Endpoint& new_endpoint);

    /**
     * @brief Add update operation to the database
     *
     * This method stores an update operation in an internal queue, being this operation then performed by a
     * dedicated thread.
     *
     * @param [in] endpoint_to_update: endpoint to update
     */
    DDSPIPE_CORE_DllAPI void update_endpoint(
            const types::Endpoint& endpoint_to_update);

    /**
     * @brief Add erase operation to the database
     *
     * This method stores an erase operation in an internal queue, being this operation then performed by a
     * dedicated thread.
     *
     * @param [in] endpoint_to_erase endpoint that will be erased
     */
    DDSPIPE_CORE_DllAPI void erase_endpoint(
            const types::Endpoint& endpoint_to_erase);

    /**
     * @brief Get the endpoint object with this guid
     *
     * @param [in] guid: guid to query
     * @return Endpoint referring to this guid
     * @throw \c InconsistencyException in case there is no entry associated to this guid
     */
    DDSPIPE_CORE_DllAPI types::Endpoint get_endpoint(
            const types::Guid& endpoint_guid) const;

    /**
     * @brief Add callback to be called when discovering an Endpoint
     *
     * @param [in] endpoint_discovered_callback: callback to add
     */
    DDSPIPE_CORE_DllAPI void add_endpoint_discovered_callback(
            std::function<void(types::Endpoint)> endpoint_discovered_callback) noexcept;

    /**
     * @brief Add callback to be called when an Endpoint has been updated
     *
     * @param [in] endpoint_updated_callback: callback to add
     */
    DDSPIPE_CORE_DllAPI void add_endpoint_updated_callback(
            std::function<void(types::Endpoint)> endpoint_updated_callback) noexcept;

    /**
     * @brief Add callback to be called when an Endpoint has been erased
     *
     * @param [in] endpoint_erased_callback: callback to add
     */
    DDSPIPE_CORE_DllAPI void add_endpoint_erased_callback(
            std::function<void(types::Endpoint)> endpoint_erased_callback) noexcept;

    /**
     * @brief Remove all callbacks from all types (endpoint discovered, updated and erased)
     *
     */
    DDSPIPE_CORE_DllAPI void clear_all_callbacks() noexcept;

protected:

    /**
     * @brief Add a new endpoint to the database.
     *
     * @param [in] new_endpoint: new endpoint to store
     * @return true if the endpoint has been added
     * @throw \c InconsistencyException in case an endpoint with the same guid already exists and is active
     */
    DDSPIPE_CORE_DllAPI bool add_endpoint_(
            const types::Endpoint& new_endpoint);

    /**
     * @brief Update an entry of the database by replacing the stored endpoint by a new one.
     *
     * @param [in] endpoint_to_update: endpoint to update
     * @return true if the endpoint has been updated
     * @throw \c InconsistencyException in case there is no entry associated to this endpoint
     */
    DDSPIPE_CORE_DllAPI bool update_endpoint_(
            const types::Endpoint& endpoint_to_update);

    /**
     * @brief Erase an endpoint inside the database
     *
     * @param [in] endpoint_to_erase endpoint that will be erased
     * @return \c RETCODE_OK if correctly erased
     * @throw \c InconsistencyException in case there is no entry associated to this endpoint
     */
    DDSPIPE_CORE_DllAPI utils::ReturnCode erase_endpoint_(
            const types::Endpoint& endpoint_to_erase);

    //! Routine performed by dedicated thread performing database operations
    DDSPIPE_CORE_DllAPI void queue_processing_thread_routine_() noexcept;

    /**
     * @brief Add new operation to the queue \c entities_to_process_
     *
     * @param [in] item: operation to add
     */
    DDSPIPE_CORE_DllAPI void push_item_to_queue_(
            std::tuple<DatabaseOperation, types::Endpoint> item) noexcept;

    //! Process queue storing database operations
    DDSPIPE_CORE_DllAPI void process_queue_() noexcept;

    //! Database of endpoints indexed by guid
    std::map<types::Guid, types::Endpoint> entities_;

    //! Mutex to guard queries to the database
    mutable std::shared_timed_mutex mutex_;

    //! Vector of callbacks to be called when an Endpoint is added
    std::vector<std::function<void(types::Endpoint)>> added_endpoint_callbacks_;

    //! Vector of callbacks to be called when an Endpoint is updated
    std::vector<std::function<void(types::Endpoint)>> updated_endpoint_callbacks_;

    //! Vector of callbacks to be called when an Endpoint is erased
    std::vector<std::function<void(types::Endpoint)>> erased_endpoint_callbacks_;

    //! Mutex to guard callbacks vectors
    mutable std::mutex callbacks_mutex_;

    //! Queue storing database operations to be performed in a dedicated thread
    fastrtps::DBQueue<std::tuple<DatabaseOperation, types::Endpoint>> entities_to_process_;

    //! Handle of thread dedicated to performing database operations
    std::thread queue_processing_thread_;

    //! Flag used to signal \c queue_processing_thread_ it must stop
    std::atomic<bool> exit_;

    //! Condition variable to wait in \c queue_processing_thread_ until a new database operation is available
    std::condition_variable entities_to_process_cv_;

    //! Guards access to \c entities_to_process_cv_
    std::mutex entities_to_process_cv_mutex_;

    //! Flag to indicate whether the DiscoveryDatabase was initialized
    std::atomic<bool> enabled_;
};

} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */
