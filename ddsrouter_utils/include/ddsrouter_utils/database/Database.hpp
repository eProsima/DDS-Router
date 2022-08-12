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

#include <memory>
#include <shared_mutex>
#include <vector>

#include <ddsrouter_utils/atomic/Atomicable.hpp>
#include <ddsrouter_utils/thread_pool/pool/SlotThreadPool.hpp>
#include <ddsrouter_utils/thread_pool/task/TaskId.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

enum class DatabaseEventType
{
    ADD,
    UPDATE,
    REMOVE,
};

template <typename Key, typename Value>
using DatabaseCallbackType = const std::function<void(Key, Value)>;

template <typename Key, typename Value>
class Database
{
public:

    ///////////////////////
    // ALIASES
    ///////////////////////

    using DatabaseType_ =
        std::map<Key, Value>;
    using CallbackRegistryType =
        std::map<
            DatabaseEventType,
            std::vector<DatabaseCallbackType<Key, Value>>>;

    ///////////////////////
    // CONSTRUCTORS
    ///////////////////////

    DiscoveryDatabase(
        std::shared_ptr<thread::SlotThreadPool> thread_pool);

    virtual ~DiscoveryDatabase();

    ///////////////////////
    // INTERNAL DATA INTERACTION METHODS
    ///////////////////////

    void add(
            const Key& key,
            Value&& value);

    void update(
            const Key& key,
            const Value& update);

    void erase(
            const Key& key);

    bool is_present(
            const Key& key);


    ///////////////////////
    // REGISTER CALLBACKS METHODS
    ///////////////////////

    void register_callback(
            const DatabaseEventType& event_type,
            const DatabaseCallbackType& callback) noexcept;

    void register_add_callback(
            const DatabaseCallbackType& callback) noexcept;

    void register_update_callback(
            const DatabaseCallbackType& callback) noexcept;

    void register_erase_callback(
            const DatabaseCallbackType& callback) noexcept;

protected:

    ///////////////////////
    // INTERNAL METHODS
    ///////////////////////

    void notify_callback_(
            const DatabaseEventType& event_type,
            const Key& key,
            const Value& value);


    ///////////////////////
    // INTERNAL VARIABLES
    ///////////////////////

    //! Database of elements indexed
    SharedAtomicable<DatabaseType_> database_;

    SharedAtomicable<CallbackRegistryType> callbacks_registry_;

    std::shared_ptr<SlotThreadPool> thread_pool_;

    std::map<DatabaseEventType, TaskId> task_ids_;
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_DYNAMIC_DISCOVERYDATABASE_HPP_ */
