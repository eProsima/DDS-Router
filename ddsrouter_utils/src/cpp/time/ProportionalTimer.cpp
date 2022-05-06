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
 * @file ProportionalTimer.cpp
 *
 */

#include <ddsrouter_utils/time/ProportionalTimer.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

ProportionalTimer::ProportionalTimer(bool activated) noexcept
    : active_timer_(activated)
{
}

void ProportionalTimer::activate() noexcept
{
    active_timer_.play();
}

void ProportionalTimer::deactivate() noexcept
{
    active_timer_.pause();
}

bool ProportionalTimer::active() const noexcept
{
    return active_timer_.playing();
}

double ProportionalTimer::elapsed() const noexcept
{
    return Timer::elapsed();
}

double ProportionalTimer::elapsed_active() const noexcept
{
    return active_timer_.elapsed();
}

double ProportionalTimer::active_proportion() const noexcept
{
    double total_elapsed = elapsed();

    if (total_elapsed)
    {
        return active_timer_.elapsed() / total_elapsed;
    }
    else
    {
        return 0.0;
    }
}

void ProportionalTimer::reset() noexcept
{
    Timer::reset();
    active_timer_.reset();
}

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */
