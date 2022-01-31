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
 * @file MultipleEventHandler.hpp
 */

#ifndef _DDSROUTER_EVENT_MULTIPLEEVENTHANDLER_HPP_
#define _DDSROUTER_EVENT_MULTIPLEEVENTHANDLER_HPP_

#include <list>
#include <memory>

#include <ddsrouter/event/EventHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

/**
 * TODO
 */
class MultipleEventHandler : public EventHandler<>
{
public:

    MultipleEventHandler(
            std::function<void()> callback);

    MultipleEventHandler();

    template <typename ... Args>
    void register_event_handler(std::unique_ptr<EventHandler<Args...>> handler) noexcept;

protected:

    std::list<std::unique_ptr<EventHandler>> handlers_registered_;
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter/event/impl/MultipleEventHandler.ipp>

#endif /* _DDSROUTER_EVENT_MULTIPLEEVENTHANDLER_HPP_ */
