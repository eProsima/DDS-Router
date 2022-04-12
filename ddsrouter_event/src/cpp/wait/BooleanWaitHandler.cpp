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
 * @file BooleanWaitHandler.cpp
 *
 */

#include <ddsrouter_utils/Log.hpp>

#include <ddsrouter_event/wait/BooleanWaitHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

BooleanWaitHandler::BooleanWaitHandler(
        bool activated /* = false */,
        bool enabled /* = true */)
    : WaitHandler<bool>(enabled, activated)
{
}

BooleanWaitHandler::~BooleanWaitHandler()
{
}

AwakeReason BooleanWaitHandler::wait(
        const utils::Duration_ms& timeout /* = 0 */)
{
    return WaitHandler<bool>::wait(
        std::function<bool(const bool&)>([this](const bool& value){return value;}),
        timeout);
}

void BooleanWaitHandler::open() noexcept
{
    set_value(true);
}

void BooleanWaitHandler::close() noexcept
{
    set_value(false);
}

bool BooleanWaitHandler::is_open() const noexcept
{
    return get_value();
}

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */
