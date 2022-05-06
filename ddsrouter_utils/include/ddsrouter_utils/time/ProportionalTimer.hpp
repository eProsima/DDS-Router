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
 * @file ProportionalTimer.hpp
 */

#ifndef _DDSROUTERUTILS_TIMER_PROPORTIONAL_TIMER_HPP_
#define _DDSROUTERUTILS_TIMER_PROPORTIONAL_TIMER_HPP_

#include <atomic>
#include <chrono>
#include <ddsrouter_utils/library/library_dll.h>
#include <ddsrouter_utils/time/PausableTimer.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

/**
 * @brief Class to measure time elapsed and allow to pause and replay it.
 *
 * This class has 2 states, paused and playing.
 * It stores internally the amount of time that has been elapsed while the timer is playing.
 *
 * @note the internal \c Timer parent class actually counts the time since the state has changed.
 *
 * @warning This class is not thread safe
 */
class ProportionalTimer : protected Timer
{
public:

    /**
     * @brief Create a new Timer which stores the time it has been created
     *
     * @param play whether the times should start playing or else paused
     */
    DDSROUTER_UTILS_DllAPI ProportionalTimer(bool activated) noexcept;

    /**
     * @brief Replay a paused Timer.
     *
     * If already playing, do nothing.
     */
    DDSROUTER_UTILS_DllAPI void activate() noexcept;

    /**
     * @brief Pause the timer. Time while paused does not count as elapsed.
     *
     * If already paused, do nothing
     */
    DDSROUTER_UTILS_DllAPI void deactivate() noexcept;

    //! Whether the timer is playing or paused
    DDSROUTER_UTILS_DllAPI bool active() const noexcept;

    DDSROUTER_UTILS_DllAPI double elapsed() const noexcept override;

    DDSROUTER_UTILS_DllAPI double elapsed_active() const noexcept;

    DDSROUTER_UTILS_DllAPI double active_proportion() const noexcept;

    /**
     * @brief  Reset timer initial time to the moment \c reset is called.
     *
     * It also reset the internal counter value, so it starts from 0 as if it was created at this moment.
     */
    DDSROUTER_UTILS_DllAPI void reset() noexcept;

protected:

    PausableTimer active_timer_;
};

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERUTILS_TIMER_PROPORTIONAL_TIMER_HPP_ */
