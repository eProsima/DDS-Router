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
 * @file MultipleEventHandler.ipp
 */

#ifndef _DDSROUTER_EVENT_IMPL_MULTIPLEEVENTHANDLER_IPP_
#define _DDSROUTER_EVENT_IMPL_MULTIPLEEVENTHANDLER_IPP_

#include <functional>

namespace eprosima {
namespace ddsrouter {
namespace event {

template <typename ... Args>
void MultipleEventHandler::register_event_handler(std::unique_ptr<EventHandler<Args...>> handler) noexcept
{
    std::function<void(Args...)> new_callback =
            [this]
                (Args... args)
            {
                // static_cast<void> (args);
                this->event_occurred_();
            };
    handler->set_callback(new_callback);

    handlers_registered_.push_back(std::move(handler));
}


} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_EVENT_IMPL_MULTIPLEEVENTHANDLER_IPP_ */
