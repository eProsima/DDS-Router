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

#include <ddsrouter_event/wait/CollectionWaitHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

/**
 *TODO
 */
template <typename T>
class DBQueueWaitHandler : public CollectionWaitHandler
{
public:

    // Make this methods public
    using CounterWaitHandler::enable;
    using CounterWaitHandler::disable;
    using CounterWaitHandler::enabled;
    using CounterWaitHandler::stop_and_continue;

protected:

    T get_next_value_() override;

    T add_value_(T&& value) override;
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTEREVENT_WAIT_DBQUEUEWAITHANDLER_HPP_ */
