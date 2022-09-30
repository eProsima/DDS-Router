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
 * @file SlotConnector.hpp
 *
 * This file contains class SlotConnector implementation.
 */

#pragma once

#include <ddsrouter_utils/thread/task/ArgsReferenceTask.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {
namespace thread {

template <typename ... Args>
SlotConnector<Args...>::SlotConnector(
        IManager* manager,
        const std::function<void(Args...)>& callback)
    : manager_(manager)
    , callback_(callback)
{
}

template <typename ... Args>
SlotConnector<Args...>::SlotConnector(
        IManager* manager,
        std::function<void(Args...)>&& callback)
    : manager_(manager)
    , callback_(std::move(callback))
{
}

template <typename ... Args>
void SlotConnector<Args...>::execute(Args... args)
{
    manager_->execute(
        std::make_unique<ArgsReferenceTask<Args...>>(
            &callback_,
            args...
        )
    );
}

} /* namespace thread */
} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */
