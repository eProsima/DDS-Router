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
 * @file DBQueueWaitHandler.hpp
 */

#ifndef _DDSROUTEREVENT_WAIT_DBQUEUEWAITHANDLER_HPP_
#define _DDSROUTEREVENT_WAIT_DBQUEUEWAITHANDLER_HPP_

#include <fastrtps/utils/DBQueue.h>

#include <ddsrouter_event/wait/CollectionWaitHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

/**
 *TODO
 */
template <typename T>
class DBQueueWaitHandler : public CollectionWaitHandler<T>
{
public:

    using CollectionWaitHandler<T>::CollectionWaitHandler;

protected:

    void add_value_(T&& value) override;

    void add_value_(const T& value) override;

    T get_next_value_() override;

    fastrtps::DBQueue<T> queue_;

    std::mutex pop_queue_mutex_;
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_event/wait/impl/DBQueueWaitHandler.ipp>

#endif /* _DDSROUTEREVENT_WAIT_DBQUEUEWAITHANDLER_HPP_ */
