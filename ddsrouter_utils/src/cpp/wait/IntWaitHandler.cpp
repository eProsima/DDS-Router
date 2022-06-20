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
 * @file IntWaitHandler.cpp
 *
 */

#include <ddsrouter_utils/Log.hpp>

#include <ddsrouter_utils/wait/IntWaitHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

IntWaitHandler::IntWaitHandler(
        IntWaitHandlerType value,
        bool enabled /* = true */)
    : WaitHandler<IntWaitHandlerType>(value, enabled)
{
}

IntWaitHandler::~IntWaitHandler()
{
}

AwakeReason IntWaitHandler::wait_equal(
        IntWaitHandlerType expected_value,
        const utils::Duration_ms& timeout /* = 0 */)
{
    return WaitHandler<IntWaitHandlerType>::wait(
        std::function<bool(const IntWaitHandlerType&)>([expected_value](const IntWaitHandlerType& value)
        {
            return value == expected_value;
        }),
        timeout);
}

AwakeReason IntWaitHandler::wait_greater_than(
        IntWaitHandlerType expected_value,
        const utils::Duration_ms& timeout /* = 0 */)
{
    return WaitHandler<IntWaitHandlerType>::wait(
        std::function<bool(const IntWaitHandlerType&)>([expected_value](const IntWaitHandlerType& value)
        {
            return value > expected_value;
        }),
        timeout);
}

AwakeReason IntWaitHandler::wait_greater_equal_than(
        IntWaitHandlerType expected_value,
        const utils::Duration_ms& timeout /* = 0 */)
{
    return WaitHandler<IntWaitHandlerType>::wait(
        std::function<bool(const IntWaitHandlerType&)>([expected_value](const IntWaitHandlerType& value)
        {
            return value >= expected_value;
        }),
        timeout);
}

AwakeReason IntWaitHandler::wait_lower_than(
        IntWaitHandlerType expected_value,
        const utils::Duration_ms& timeout /* = 0 */)
{
    return WaitHandler<IntWaitHandlerType>::wait(
        std::function<bool(const IntWaitHandlerType&)>([expected_value](const IntWaitHandlerType& value)
        {
            return value < expected_value;
        }),
        timeout);
}

AwakeReason IntWaitHandler::wait_lower_equal_than(
        IntWaitHandlerType expected_value,
        const utils::Duration_ms& timeout /* = 0 */)
{
    return WaitHandler<IntWaitHandlerType>::wait(
        std::function<bool(const IntWaitHandlerType&)>([expected_value](const IntWaitHandlerType& value)
        {
            return value <= expected_value;
        }),
        timeout);
}

IntWaitHandler& IntWaitHandler::operator ++()
{
    // NOTE: This operation could be done using the WaitHandler methods, but it will be less efficient than
    // set the actual value directly.

    {
        // Mutex must guard the modification of value_
        std::lock_guard<std::mutex> lock(wait_condition_variable_mutex_);
        value_++;
    }

    wait_condition_variable_.notify_all();

    return *this;
}

IntWaitHandler& IntWaitHandler::operator --()
{
    // NOTE: This operation could be done using the WaitHandler methods, but it will be less efficient than
    // set the actual value directly.

    {
        // Mutex must guard the modification of value_
        std::lock_guard<std::mutex> lock(wait_condition_variable_mutex_);
        value_--;
    }

    wait_condition_variable_.notify_all();

    return *this;
}

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */
