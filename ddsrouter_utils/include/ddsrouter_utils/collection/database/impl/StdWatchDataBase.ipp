// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file StdWatchDataBase.ipp
 */

#pragma once

#include <ddsrouter_utils/exception/InconsistencyException.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

template <typename Key, typename Value>
StdWatchDataBase<Key, Value>::StdWatchDataBase(
        std::shared_ptr<thread::IManager> thread_manager)
    : thread_manager_(thread_manager)
    , add_slot_connector_(
        thread::SlotConnector<Key, Value>(
            thread_manager.get(),
            [this](Key key, Value value){ this->add(key, value); }))
    , modify_slot_connector_(
        thread::SlotConnector<Key, Value>(
            thread_manager.get(),
            [this](Key key, Value value){ this->modify(key, value); }))
    , remove_slot_connector_(
        thread::SlotConnector<Key>(
            thread_manager.get(),
            [this](Key key){ this->remove(key); }))
{
}

template <typename Key, typename Value>
void StdWatchDataBase<Key, Value>::add(const Key& key, const Value& value)
{
    {
        std::unique_lock<std::shared_timed_mutex> lock_db(map_database_);
        auto it = map_database_.find(key);
        if (it != map_database_.end())
        {
            throw InconsistencyException(
                STR_ENTRY << "Value to add " << key << " already in Database.");
        }
        map_database_[key] = value;
    }

    call_callback_common_(key, value, DataBaseActionKind::add);
}

template <typename Key, typename Value>
void StdWatchDataBase<Key, Value>::modify(const Key& key, const Value& value)
{
    {
        std::unique_lock<std::shared_timed_mutex> lock_db(map_database_);
        auto it = map_database_.find(key);
        if (it == map_database_.end())
        {
            throw InconsistencyException(
                STR_ENTRY << "Value to modify " << key << " not in Database.");
        }
        map_database_[key] = value;
    }

    call_callback_common_(key, value, DataBaseActionKind::modify);
}

template <typename Key, typename Value>
void StdWatchDataBase<Key, Value>::remove(const Key& key)
{
    Value value;

    {
        std::unique_lock<std::shared_timed_mutex> lock_db(map_database_);
        auto it = map_database_.find(key);
        if (it == map_database_.end())
        {
            throw InconsistencyException(
                STR_ENTRY << "Value to remove " << key << " not in Database.");
        }
        value = it->second;
        map_database_.erase(it);
    }

    call_callback_common_(key, value, DataBaseActionKind::remove);
}

template <typename Key, typename Value>
bool StdWatchDataBase<Key, Value>::exist(const Key& key)
{
    std::shared_lock<std::shared_timed_mutex> lock_db(map_database_);
    return map_database_.find(key) == map_database_.end();
}

template <typename Key, typename Value>
Value StdWatchDataBase<Key, Value>::get(const Key& key)
{
    std::shared_lock<std::shared_timed_mutex> lock_db(map_database_);
    auto it = map_database_.find(key);
    if (it == map_database_.end())
    {
        throw InconsistencyException(
            STR_ENTRY << "Value to get " << key << " not in Database.");
    }
    return it->second;
}

template <typename Key, typename Value>
void StdWatchDataBase<Key, Value>::register_addition_callback(
        const CallbackType& callback)
{
    register_callback_common_(callback, DataBaseActionKind::add);
}

template <typename Key, typename Value>
void StdWatchDataBase<Key, Value>::register_modification_callback(
        const CallbackType& callback)
{
    register_callback_common_(callback, DataBaseActionKind::modify);
}

template <typename Key, typename Value>
void StdWatchDataBase<Key, Value>::register_deletion_callback(
        const CallbackType& callback)
{
    register_callback_common_(callback, DataBaseActionKind::remove);
}

template <typename Key, typename Value>
void StdWatchDataBase<Key, Value>::async_add(Key key, Value value)
{
    add_slot_connector_.execute(key, value);
}

template <typename Key, typename Value>
void StdWatchDataBase<Key, Value>::async_modify(Key key, Value value)
{
    modify_slot_connector_.execute(key, value);
}

template <typename Key, typename Value>
void StdWatchDataBase<Key, Value>::async_remove(Key key)
{
    remove_slot_connector_.execute(key);
}

template <typename Key, typename Value>
void StdWatchDataBase<Key, Value>::call_callbacks_common_(
        const Key& key,
        const Value& value,
        DataBaseActionKind action_kind,
        bool sync)
{
    std::shared_lock<std::shared_timed_mutex> lock_db(callbacks_[action_kind]);
    for (auto& slot : callbacks_[action_kind])
    {
        slot.execute(key, value);
    }
}

template <typename Key, typename Value>
void StdWatchDataBase<Key, Value>::register_callback_common_(
        const CallbackType& callback,
        DataBaseActionKind action_kind)
{
    std::unique_lock<std::shared_timed_mutex> lock_db(callbacks_[action_kind]);
    callbacks_[action_kind].push_back(
        thread::SlotConnector<Key, Value>(
            thread_manager_.get(),
            callback));
}

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */
