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
 * @file EventHandler.ipp
 */

#ifndef _DDSROUTER_USERINTERFACE_IMPL_HANDLER_IPP_
#define _DDSROUTER_USERINTERFACE_IMPL_HANDLER_IPP_

#include <functional>

#include <ddsrouter/types/Log.hpp>

namespace eprosima {
namespace ddsrouter {
namespace interface {

template <class T>
const std::function<void(T)> EventHandler<T>::DEFAULT_CALLBACK_ =
        [](T t)
        {
            logWarning(DDSROUTER_HANDLER, "Calling unset callback with arg: " << t << ".");
        };

template <class T>
void EventHandler<T>::set_callback(
        std::function<void(T)> callback) noexcept
{
    callback_ = callback;
}

template <class T>
void EventHandler<T>::unset_callback() noexcept
{
    callback_ = DEFAULT_CALLBACK_;
}

template <class T>
void EventHandler<T>::call_callback_(T arg) noexcept
{
    callback_(arg);
}

} /* namespace interface */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_USERINTERFACE_IMPL_HANDLER_IPP_ */
