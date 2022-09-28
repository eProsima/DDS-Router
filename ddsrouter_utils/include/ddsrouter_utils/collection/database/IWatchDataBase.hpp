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
 * @file IWatchDataBase.hpp
 */

#pragma once

#include <functional>

#include <ddsrouter_utils/collection/database/IDataBase.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

template <typename Key, typename Value>
class IWatchDataBase : public IDataBase<Key, Value>
{
public:

    virtual void register_addition_callback(
        const std::function<void(Key, Value)>& callback) = 0;

    virtual void register_modification_callback(
        const std::function<void(Key, Value)>& callback) = 0;

    virtual void register_remove_callback(
        const std::function<void(Key, Value)>& callback) = 0;

};

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */
