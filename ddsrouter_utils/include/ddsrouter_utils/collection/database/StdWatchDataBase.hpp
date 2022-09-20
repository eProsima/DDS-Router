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
#include <ddsrouter_utils/types/Atomicable.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

enum DataBaseActionKind
{
    add = 0,
    modify = 1,
    remove = 2
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

    virtual void register_deletion_callback(
            const CallbackType& callback) override;

protected:

    void call_callback_common_(
            const Key& key,
            const Value& value,
            DataBaseActionKind action_kind);

    void register_callback_common_(
            const CallbackType& callback,
            DataBaseActionKind action_kind);

    std::shared_ptr<thread::IManager> thread_manager_;

    SharedAtomicable<std::map<Key, Value>> map_database_;

    std::array<SharedAtomicable<std::vector<CallbackType>>, 3> callbacks_;

};

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_utils/collection/database/impl/StdWatchDataBase.ipp>
