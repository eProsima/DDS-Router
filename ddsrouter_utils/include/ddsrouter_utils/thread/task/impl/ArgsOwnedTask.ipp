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
 * @file ArgsOwnedTask.ipp
 *
 * This file contains class OneShotConnector implementation.
 */

#pragma once

namespace eprosima {
namespace ddsrouter {
namespace utils {
namespace thread {

template <typename ... Args>
ArgsOwnedTask<Args...>::ArgsOwnedTask(
        std::function<void (Args...)> callback,
        const Args&... args)
    : callback_(callback)
    , args_(args...)
{
}

template <typename ... Args>
void ArgsOwnedTask<Args...>::operator()() noexcept
{
    call_internal_callback_(helper::gen_seq<sizeof...(Args)>{});
}

// template <typename ... Args, std::size_t... Is>
// void ArgsOwnedTask<Args...>::call_internal_callback_(helper::index<Is...>)
// {
//     callback_(std::get<Is>(args_)...);
// }

} /* namespace thread */
} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */
