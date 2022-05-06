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
 * @file PausableTimer.hpp
 */

#ifndef _DDSROUTERUTILS_PAUSABLETIMER_HPP_
#define _DDSROUTERUTILS_PAUSABLETIMER_HPP_

#include <atomic>
#include <chrono>
#include <ddsrouter_utils/library/library_dll.h>
#include <ddsrouter_utils/time/Timer.hpp>

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
 */
class PausableTimer : public Timer
{
public:

    /**
     * @brief Create a new Timer which stores the time it has been created
     *
     * @param play whether the times should start playing or else paused
     */
    DDSROUTER_UTILS_DllAPI PausableTimer(bool play) noexcept;

    /**
     * @brief Replay a paused Timer.
     *
     * If already playing, do nothing.
     */
    DDSROUTER_UTILS_DllAPI void play() noexcept;

    /**
     * @brief Pause the timer. Time while paused does not count as elapsed.
     *
     * If already paused, do nothing
     */
    DDSROUTER_UTILS_DllAPI void pause() noexcept;

    //! Elapsed time in milliseconds with object playing
    DDSROUTER_UTILS_DllAPI double elapsed() const noexcept override;

    /**
     * @brief  Reset timer initial time to the moment \c reset is called.
     *
     * It also reset the internal counter value, so it starts from 0 as if it was created at this moment.
     */
    DDSROUTER_UTILS_DllAPI void reset() noexcept override;

    //! Whether the timer is playing or paused
    DDSROUTER_UTILS_DllAPI bool playing() const noexcept;

protected:

    /**
     * @brief Reset the parent class timer but do not reset stored time.
     *
     * This method is used when timer is paused, so the timer indicates the time since last
     * pause / play has been called, but the stored timer is not affected.
     */
    void reset_() noexcept;

    //! Whether the timer is paused or not
    std::atomic<bool> playing_;

    /**
     * @brief Time elapsed while object has been playing
     *
     * This object is used to store the time it has been elapsed while object
     * has being playing in previous plays. It does not store the current time if currently playing.
     *
     * This value (plus the amount of time elapsed in case of currently playing) is the amount of time elapsed.
     */
    double stored_time_elapsed_;
};

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERUTILS_PAUSABLETIMER_HPP_ */
