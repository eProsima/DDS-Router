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
 * @file Time.cpp
 *
 */

#include <ddsrouter_utils/Time.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

Timestamp now() noexcept
{
    return std::chrono::system_clock::now();
}

Timestamp the_end_of_times() noexcept
{
    return  std::chrono::time_point<std::chrono::system_clock>::max();
}

std::chrono::milliseconds duration_to_ms(
        const Duration_ms& duration) noexcept
{
    return std::chrono::milliseconds(duration);
}

Timer::Timer() noexcept
    : start_time_(std::chrono::high_resolution_clock::now())
{
}

void Timer::reset() noexcept
{
    start_time_ = std::chrono::high_resolution_clock::now();
}

double Timer::elapsed() const noexcept
{
    std::chrono::time_point<std::chrono::high_resolution_clock> now_time = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(now_time - start_time_).count();
}

Duration_ms Timer::elapsed_ms() const noexcept
{
    double elapsed_time = elapsed();
    return static_cast<Duration_ms>(elapsed_time);
}

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */
