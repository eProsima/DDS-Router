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
 * @file DataBase.hpp
 */

#pragma once

#include <map>

#include <ddsrouter_utils/collection/database/IWatchDataBase.hpp>
#include <ddsrouter_utils/thread/manager/IManager.hpp>
#include <ddsrouter_utils/thread/connector/SlotConnector.hpp>
#include <ddsrouter_utils/types/Atomicable.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

enum DataBaseActionKind
{
    add = 0,
    modify = 1,
    remove = 2,
    enum_size = 3,
};

template <typename Key, typename Value>
class StdWatchDataBase : public IWatchDataBase<Key, Value>
{

    using CallbackType = std::function<void(Key, Value)>;

public:

    StdWatchDataBase(
            std::shared_ptr<thread::IManager> thread_manager);

    virtual ~StdWatchDataBase() = default;

    virtual void add(const Key& key, const Value& value) override;

    virtual void modify(const Key& key, const Value& value) override;

    virtual void remove(const Key& key) override;

    virtual bool exist(const Key& key) override;

    virtual Value get(const Key& key) override;

    virtual void register_addition_callback(
            const CallbackType& callback) override;

    virtual void register_modification_callback(
            const CallbackType& callback) override;

    virtual void register_remove_callback(
            const CallbackType& callback) override;

    virtual void add(Key key, Value value, bool sync_insertion, bool sync_callback_call, bool sync_all_callbacks);

    virtual void modify(Key key, Value value, bool sync_insertion, bool sync_callback_call, bool sync_all_callbacks);

    virtual void remove(Key key, bool sync_insertion, bool sync_callback_call, bool sync_all_callbacks);

protected:

    void sync_add_(const Key& key, const Value& value);
    void sync_modify_(const Key& key, const Value& value);
    void sync_remove_(const Key& key, const Value& value);

    void sync_call_callbacks_common_(
            const Key& key,
            const Value& value,
            DataBaseActionKind action_kind);

    void call_callbacks_common_sync_(
            const Key& key,
            const Value& value,
            DataBaseActionKind action_kind);

    void register_callback_common_(
            const CallbackType& callback,
            DataBaseActionKind action_kind);

    std::shared_ptr<thread::IManager> thread_manager_;

    SharedAtomicable<std::map<Key, Value>> map_database_;

    std::array<
        SharedAtomicable<
            std::vector<
                thread::SlotConnector<Key, Value>>>,
                DataBaseActionKind::enum_size> callbacks_;

    thread::SlotConnector<Key, Value> add_slot_connector_;
    thread::SlotConnector<Key, Value> modify_slot_connector_;
    thread::SlotConnector<Key> remove_slot_connector_;
    thread::SlotConnector<Key, Value, DataBaseActionKind> call_callbacks_slot_connector_;

};

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_utils/collection/database/impl/StdWatchDataBase.ipp>
