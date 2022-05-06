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
 * @file time_utils.cpp
 *
 */

#include <thread>

#include <ddsrouter_utils/time/time_utils.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

Timestamp now() noexcept
{
    return std::chrono::system_clock::now();
}

Timestamp the_end_of_times() noexcept
{
    return std::chrono::time_point<std::chrono::system_clock>::max();
}

std::chrono::milliseconds duration_to_ms(
        const Duration_ms& duration) noexcept
{
    return std::chrono::milliseconds(duration);
}

void sleep_for(
        const Duration_ms& sleep_time) noexcept
{
    std::this_thread::sleep_for(duration_to_ms(sleep_time));
}

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */
