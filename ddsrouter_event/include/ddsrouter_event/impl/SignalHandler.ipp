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
 * @file SignalHandler.ipp
 *
 */

#ifndef _DDSROUTEREVENT_IMPL_SIGNALHANDLER_IPP_
#define _DDSROUTEREVENT_IMPL_SIGNALHANDLER_IPP_

#include <algorithm>

#include <ddsrouter_utils/Log.hpp>

#include <ddsrouter_event/SignalManager.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

template <int SigNum>
SignalHandler<SigNum>::SignalHandler() noexcept
    : SignalHandler<SigNum>(
        [](int signum)
        {
            logInfo(DDSROUTER_SIGNALHANDLER,
            "Received signal " << signum << " in specific handler.");
        })
{
}

template <int SigNum>
SignalHandler<SigNum>::SignalHandler(
        std::function<void(int)> callback) noexcept
    : EventHandler<int>()
    , callback_set_in_manager_(false)
{
    set_callback(callback);
    logDebug(DDSROUTER_SIGNALHANDLER, "SignalHandler created for signal: " << SigNum << ".");
}

template <int SigNum>
SignalHandler<SigNum>::~SignalHandler()
{
    unset_callback();
    logDebug(DDSROUTER_SIGNALHANDLER, "SignalHandler destroyed for signal: " << SigNum << ".");
}

template <int SigNum>
void SignalHandler<SigNum>::callback_set_nts_() noexcept
{
    if (!callback_set_in_manager_.exchange(true))
    {
        callback_id_ = SignalManager<SigNum>::get_instance().add_callback(
                std::bind(&SignalHandler<SigNum>::signal_received_callback_, this)
            );
        // callback_id_ = SignalManager<SigNum>::get_instance().add_callback(
        //     [](int s){ std::cout << "!!" << s << std::endl; });
    }
}

template <int SigNum>
void SignalHandler<SigNum>::callback_unset_nts_() noexcept
{
    if (callback_set_in_manager_.exchange(false))
    {
        SignalManager<SigNum>::get_instance().erase_callback(callback_id_);
    }
}

template <int SigNum>
void SignalHandler<SigNum>::signal_received_callback_() noexcept
{
    event_occurred_(SigNum);
}

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTEREVENT_IMPL_SIGNALHANDLER_IPP_ */
