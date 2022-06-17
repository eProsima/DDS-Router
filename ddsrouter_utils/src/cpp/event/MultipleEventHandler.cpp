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
 * @file MultipleEventHandler.cpp
 *
 */

#include <ddsrouter_utils/event/MultipleEventHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

MultipleEventHandler::MultipleEventHandler(
        std::function<void()> callback)
    : EventHandler()
{
    set_callback(callback);
}

MultipleEventHandler::MultipleEventHandler()
    : MultipleEventHandler([]()
            {
                // Default callback when it is not set
                logDebug(DDSROUTER_MULTIPLEEVENTHANDLER,
                "Event received in MultipleEventHandler.");
            })
{
}

MultipleEventHandler::~MultipleEventHandler()
{
    // Destroy every object inside before this is destroyed, so in case a callback arise,
    // it does not call a deleted object
    for (std::unique_ptr<IBaseEventHandler>& event : handlers_registered_)
    {
        event.reset();
    }

    // At this point the callback is not recheable, but it should be unset before destroying the object
    unset_callback();

    logDebug(DDSROUTER_MULTIPLEEVENTHANDLER,
            "MultipleEventHandler has been destroyed.");
}

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */
