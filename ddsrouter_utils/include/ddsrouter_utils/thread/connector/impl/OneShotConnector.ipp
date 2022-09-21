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
 * @file OneShotConnector.hpp
 *
 * This file contains class OneShotConnector implementation.
 */

#pragma once

#include <ddsrouter_utils/thread/task/ArgsOwnedTask.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {
namespace thread {

template <typename ... Args>
void OneShotConnector<Args...>::execute(
        IManager* manager,
        const std::function<void(Args...)>& callback,
        Args... args)
{
    manager->execute(
        std::make_unique<ArgsOwnedTask<Args...>>(
            callback,
            args...
        )
    );
}

template <typename ... Args>
void OneShotConnector<Args...>::execute(
        IManager* manager,
        std::function<void(Args...)>&& callback,
        Args... args)
{
    manager->execute(
        std::make_unique<ArgsOwnedTask<Args...>>(
            std::move(callback),
            args...
        )
    );
}

// template <typename ... Args>
// void OneShotConnector<Args...>::execute(
//         IManager* manager,
//         std::function<void(Args...)> callback,
//         Args... args)
// {
//     manager->execute(
//         std::make_unique<ArgsOwnedTask<Args...>>(
//             callback,
//             args...
//         )
//     );
// }

} /* namespace thread */
} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */
