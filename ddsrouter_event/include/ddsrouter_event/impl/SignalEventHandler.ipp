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
 * @file SignalEventHandler.ipp
 *
 */

#ifndef _DDSROUTEREVENT_IMPL_SIGNALEVENTHANDLER_IPP_
#define _DDSROUTEREVENT_IMPL_SIGNALEVENTHANDLER_IPP_

#include <algorithm>

#include <ddsrouter_utils/Log.hpp>

#include <ddsrouter_event/SignalManager.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

template <Signal SigVal>
SignalEventHandler<SigVal>::SignalEventHandler() noexcept
    : SignalEventHandler<SigVal>(
        [](Signal sigval)
        {
            logInfo(DDSROUTER_SIGNALHANDLER,
            "Received signal " << sigval << " in specific handler.");
        })
{
}

template <Signal SigVal>
SignalEventHandler<SigVal>::SignalEventHandler(
        std::function<void(Signal)> callback) noexcept
    : EventHandler<Signal>()
    , callback_set_in_manager_(false)
{
    set_callback(callback);
    logDebug(DDSROUTER_SIGNALHANDLER, "SignalEventHandler created for signal: " << SigVal << ".");
}

template <Signal SigVal>
SignalEventHandler<SigVal>::~SignalEventHandler()
{
    unset_callback();
    logDebug(DDSROUTER_SIGNALHANDLER, "SignalEventHandler destroyed for signal: " << SigVal << ".");
}

template <Signal SigVal>
void SignalEventHandler<SigVal>::callback_set_nts_() noexcept
{
    if (!callback_set_in_manager_.exchange(true))
    {
        callback_id_ = SignalManager<SigVal>::get_instance().register_callback(
            std::bind(&SignalEventHandler<SigVal>::signal_received_callback_, this)
            );
    }
}

template <Signal SigVal>
void SignalEventHandler<SigVal>::callback_unset_nts_() noexcept
{
    if (callback_set_in_manager_.exchange(false))
    {
        SignalManager<SigVal>::get_instance().unregister_callback(callback_id_);
    }
}

template <Signal SigVal>
void SignalEventHandler<SigVal>::signal_received_callback_() noexcept
{
    event_occurred_(SigVal);
}

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTEREVENT_IMPL_SIGNALEVENTHANDLER_IPP_ */
