// Copyright 2022
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
 * @file ConsumerWaitHandler.ipp
 */

#include <ddsrouter_utils/exception/DisabledException.hpp>
#include <ddsrouter_utils/exception/TimeoutException.hpp>
#include <ddsrouter_utils/Log.hpp>

#ifndef _DDSROUTEREVENT_WAIT_IMPL_CONSUMERWAITHANDLER_IPP_
#define _DDSROUTEREVENT_WAIT_IMPL_CONSUMERWAITHANDLER_IPP_

namespace eprosima {
namespace ddsrouter {
namespace event {

template <typename T>
ConsumerWaitHandler<T>::ConsumerWaitHandler(
        CounterType initial_value /* = 0 */,
        bool enabled /* = true */)
    : CounterWaitHandler(0, initial_value, enabled)
{
    logDebug(DDSROUTER_WAIT_CONSUMER, "Created Consumer Wait Handler with type " << TYPE_NAME(T) << ".");
}

template <typename T>
CounterType ConsumerWaitHandler<T>::elements_ready_to_consume() const noexcept
{
    return get_value();
}

template <typename T>
void ConsumerWaitHandler<T>::produce(
        T&& value)
{
    add_value_(std::move(value));
    this->operator ++();
}

template <typename T>
void ConsumerWaitHandler<T>::produce(
        const T& value)
{
    T dummy_copied_value__(value);
    add_value_(std::move(dummy_copied_value__));
    this->operator ++();
}

template <typename T>
T ConsumerWaitHandler<T>::consume(
        const utils::Duration_ms& timeout /* = 0 */)
{
    AwakeReason reason = wait_and_decrement(timeout);

    // Check if reason has been condition met, else throw exception
    if (reason == AwakeReason::disabled)
    {
        throw utils::DisabledException("ConsumerWaitHandler has been disabled.");
    }
    else if (reason == AwakeReason::timeout)
    {
        throw utils::TimeoutException("ConsumerWaitHandler awaken by timeout.");
    }
    else
    {
        // This is taken without mutex protection
        return get_next_value_();
    }
}

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTEREVENT_WAIT_IMPL_CONSUMERWAITHANDLER_IPP_ */
