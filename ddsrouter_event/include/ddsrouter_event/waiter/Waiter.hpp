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
 * @file Waiter.hpp
 */

#ifndef _DDSROUTEREVENT_WAITER_WAITER_HPP_
#define _DDSROUTEREVENT_WAITER_WAITER_HPP_

#include <atomic>
#include <condition_variable>
#include <mutex>

#include <ddsrouter_utils/Time.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

enum AwakeReason
{
    CONDITION_MET,
    DEACTIVATED,
    TIMEOUT
};

class Waiter
{
public:

    Waiter(
        bool enabled = true);

    ~Waiter();

    /////
    // Enabling methods

    void enable() noexcept;

    void disable() noexcept;

    bool enabled() const noexcept;

    /////
    // Wait methods

    virtual AwakeReason wait(
            const utils::Duration_ms& timeout = 0);

    /////
    // Awake methods

    void awake_all() const noexcept;

    void awake_one() const noexcept;

protected:

    std::atomic<bool> enabled_;

    std::atomic<uint32_t> should_awake_;

    std::atomic<uint32_t> threads_waiting_;

    std::condition_variable wait_condition_variable_;

    std::mutex wait_condition_variable_mutex_;
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_event/impl/EventHandler.ipp>

#endif /* _DDSROUTEREVENT_WAITER_WAITER_HPP_ */
