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
 * @file Timer.hpp
 */

#ifndef _DDSROUTERUTILS_TIMER_HPP_
#define _DDSROUTERUTILS_TIMER_HPP_

#include <atomic>
#include <chrono>
#include <ddsrouter_utils/library/library_dll.h>
#include <ddsrouter_utils/time/time_utils.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

/**
 * @brief Class to measure time elapsed.
 *
 * When creating an object of this class stores the time \c now when it is created.
 * Using method \c elapsed / \c elapsed_ms gives the amount of time elapsed since its creation.
 */
class Timer
{
public:

    //! @brief Create a new Timer which stores the time it has been created
    DDSROUTER_UTILS_DllAPI Timer() noexcept;

    //! Reset timer initial time to the moment \c reset is called.
    DDSROUTER_UTILS_DllAPI virtual void reset() noexcept;

    //! Elapsed time in milliseconds since beggining or last reset
    DDSROUTER_UTILS_DllAPI virtual double elapsed() const noexcept;

    /**
     * @brief Elapsed time rounded to milliseconds
     *
     * @note Derived method: do not need to be overriden
     *
     * @return number of milliseconds elapsed since beggining or last reset
     */
    DDSROUTER_UTILS_DllAPI Duration_ms elapsed_ms() const noexcept;

protected:

    //! Time when the object has been created OR when last reset has been called.
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
};

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERUTILS_TIMER_HPP_ */
