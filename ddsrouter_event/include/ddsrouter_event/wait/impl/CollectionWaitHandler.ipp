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
 * @file CollectionWaitHandler.ipp
 */

#include <ddsrouter_utils/exception/DisabledException.hpp>
#include <ddsrouter_utils/exception/TimeoutException.hpp>
#include <ddsrouter_utils/Log.hpp>

#ifndef _DDSROUTEREVENT_WAIT_IMPL_COLLECTIONWAITHANDLER_IPP_
#define _DDSROUTEREVENT_WAIT_IMPL_COLLECTIONWAITHANDLER_IPP_

namespace eprosima {
namespace ddsrouter {
namespace event {

template <typename T>
CollectionWaitHandler<T>::CollectionWaitHandler(
        bool enabled /* = true */)
    : CounterWaitHandler(0, enabled)
{
    logDebug(DDSROUTER_WAIT_COLLECTION, "Created Collection Wait Handler with type " << TYPE_NAME(T) << ".");
}

template <typename T>
void CollectionWaitHandler<T>::add_value(T&& value)
{
    add_value_(std::move(value));
    this->operator++();
}

template <typename T>
void CollectionWaitHandler<T>::add_value(const T& value)
{
    add_value_(value);
    this->operator++();
}

template <typename T>
T CollectionWaitHandler<T>::wait_next_value(
        const utils::Duration_ms& timeout /* = 0 */)
{
    {
        // Do wait with mutex taken
        std::unique_lock<std::mutex> lock(wait_condition_variable_mutex_);

        // Check if it is disabled and exit
        if (!enabled())
        {
            throw utils::DisabledException("CollectionWaitHandler disabled.");
        }

        // Increment number of threads waiting
        // WARNING: mutex must be taken
        threads_waiting_++;

        utils::Timestamp time_to_wait_until;

        // If timeout is 0, use wait, if not use wait for timeout
        if (timeout > 0)
        {
            time_to_wait_until = utils::now() + utils::duration_to_ms(timeout);
        }
        else
        {
            time_to_wait_until = utils::the_end_of_times();
        }

        bool finished_for_condition_met = wait_condition_variable_.wait_until(
            lock,
            time_to_wait_until,
            [this]
            {
                // Exit if predicate is true or if this has been disabled
                return !enabled_.load() || value_ > 0;
            });

        // Decrement number of threads waiting
        // NOTE: mutex is still taken
        threads_waiting_--;

        // Check awake reason. Mutex is taken so it can not change while checking
        if (!enabled_.load())
        {
            throw utils::DisabledException("CollectionWaitHandler has been disabled.");
        }
        else if (!finished_for_condition_met)
        {
            throw utils::TimeoutException("CollectionWaitHandler awaken by timeout.");
        }
        else
        {
            // Reduce the number of the value of counter manually, as the mutex is already taken
            // TODO implement nts methods in parents so this is no so dirty.
            value_--;
        }
    }

    // Once the mutex is released, take the value from the internal collection
    // TODO: rethink if it is better to do this without handler mutex
    return get_next_value_();
}



} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTEREVENT_WAIT_IMPL_COLLECTIONWAITHANDLER_IPP_ */
