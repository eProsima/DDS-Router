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
 * @file CounterWaitHandler.cpp
 *
 */

#include <ddsrouter_utils/Log.hpp>

#include <ddsrouter_utils/wait/CounterWaitHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

CounterWaitHandler::CounterWaitHandler(
        CounterType threshold,
        CounterType initial_value,
        bool enabled /* = true */)
    : WaitHandler<CounterType>(initial_value, enabled)
    , threshold_(threshold)
{
}

CounterWaitHandler::~CounterWaitHandler()
{
}

AwakeReason CounterWaitHandler::wait_and_decrement(
        const utils::Duration_ms& timeout /* = 0 */) noexcept
{
    AwakeReason result; // Get value from wait
    CounterType threshold_tmp = threshold_; // Require to set it in predicate

    // Perform blocking wait
    auto lock = blocking_wait_(
        std::function<bool(const CounterType&)>([threshold_tmp](const CounterType& value)
        {
            return value > threshold_tmp;
        }),
        timeout,
        result);

    // Mutex is taken, decrease value by 1 if condition was met
    if (result == AwakeReason::condition_met)
    {
        decrease_1_nts_();
    }

    return result;
}

CounterWaitHandler& CounterWaitHandler::operator ++()
{
    // NOTE: This operation could be done using the WaitHandler methods, but it will be less efficient than
    // set the actual value directly.

    {
        // Mutex must guard the modification of value_
        std::lock_guard<std::mutex> lock(wait_condition_variable_mutex_);
        value_++;

        // If threshold is reached, notify one waiter
        if (value_ > threshold_)
        {
            wait_condition_variable_.notify_one();
        }
    }

    return *this;
}

void CounterWaitHandler::decrease_1_nts_()
{
    value_--;

    // If value is still higher than threshold, notify one waiter
    if (value_ > threshold_)
    {
        wait_condition_variable_.notify_one();
    }
}

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */
