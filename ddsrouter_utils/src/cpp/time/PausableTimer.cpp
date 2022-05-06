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

#include <ddsrouter_utils/time/PausableTimer.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

PausableTimer::PausableTimer(bool play) noexcept
    : playing_(play)
    , stored_time_elapsed_(0)
{
}

void PausableTimer::play() noexcept
{
    // If object was paused, reactivate the counter
    if (!playing_.exchange(true))
    {
        reset_();
    }

    // If it was already playing, do nothing
}

void PausableTimer::pause() noexcept
{
    // If object was playing, store the amount of data elapsed and restore the counter
    if (playing_.exchange(false))
    {
        stored_time_elapsed_ += Timer::elapsed();
        reset_();
    }

    // If it was already paused, do nothing
}

// WOW! This method was written by copilot. First time I saw it doing something intellectually "complex"
double PausableTimer::elapsed() const noexcept
{
    // If object is paused, return the amount of data elapsed
    if (!playing_.load())
    {
        return stored_time_elapsed_;
    }
    else
    {
        // Otherwise, return the amount of data elapsed since the object was created
        return stored_time_elapsed_ + Timer::elapsed();
    }
}

void PausableTimer::reset() noexcept
{
    stored_time_elapsed_ = 0;
    reset_();
}

bool PausableTimer::playing() const noexcept
{
    return playing_.load();
}

void PausableTimer::reset_() noexcept
{
    Timer::reset();
}

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */
