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
 * @file ArgsOwnedTask.hpp
 *
 * This file contains class Task definition.
 */

#pragma once

#include <functional>
#include <tuple>

#include <ddsrouter_utils/thread/task/ITask.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {
namespace thread {

namespace helper
{
    template <std::size_t... Ts>
    struct index {};

    template <std::size_t N, std::size_t... Ts>
    struct gen_seq : gen_seq<N - 1, N - 1, Ts...> {};

    template <std::size_t... Ts>
    struct gen_seq<0, Ts...> : index<Ts...> {};
}

template <typename... Args>
class ArgsOwnedTask : public ITask
{
public:

    ArgsOwnedTask(
            std::function<void (Args...)> callback,
            const Args&... args);

    void operator()() noexcept override;

protected:

    template <std::size_t... Is>
    void call_internal_callback_(helper::index<Is...>)
    {
        callback_(std::get<Is>(args_)...);
    }

    std::function<void (Args...)> callback_;
    std::tuple<Args...> args_;

};


// template <typename ... Args>
// class ArgsOwnedTask : public ITask
// {
// public:

//     ArgsOwnedTask(
//         const std::function<void(Args...)>& callback,
//         const Args&... args);

//     // ArgsOwnedTask(
//     //     std::function<void(Args...)>&& callback,
//     //     Args... args);

//     virtual ~ArgsOwnedTask() = default;

//     template <int... Ts>
//     void call_(helper::index<Ts...>)
//     {
//         callback_(std::get<Ts>(args_)...);
//     }

//     virtual void operator()() noexcept override
//     {
//         call_(helper::gen_seq<sizeof...(Args)>{});
//     }

// protected:

//     std::function<void(Args...)> callback_;

//     std::tuple<Args...> args_;
// };

} /* namespace thread */
} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_utils/thread/task/impl/ArgsOwnedTask.ipp>
