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
 * @file SlotConnector.hpp
 *
 * This file contains class SlotConnector definition.
 */

#pragma once

#include <functional>

#include <ddsrouter_utils/thread/task/ITask.hpp>
#include <ddsrouter_utils/thread/manager/IManager.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {
namespace thread {

template <typename ... Args>
class SlotConnector
{
public:

    SlotConnector(
        IManager* manager,
        const std::function<void(Args...)>& callback);

    SlotConnector(
        IManager* manager,
        std::function<void(Args...)>&& callback);

    ~SlotConnector() = default;

    void execute(Args...);

protected:

    IManager* manager_;

    std::function<void (Args...)> callback_;

};
using SimpleSlotConnector = SlotConnector<>;

} /* namespace thread */
} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_utils/thread/connector/impl/SlotConnector.ipp>
