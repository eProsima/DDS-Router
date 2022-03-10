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
 * @file Timestamp.hpp
 */

#ifndef _DDSROUTERUTILS_TIME_HPP_
#define _DDSROUTERUTILS_TIME_HPP_

#include <chrono>
#include <ddsrouter_utils/library/library_dll.h>

namespace eprosima {
namespace ddsrouter {
namespace utils {

//! Type of Duration in milliseconds
using Duration_ms = uint32_t;

/**
 * Type used to represent time points
 */
using Timestamp = std::chrono::time_point<std::chrono::system_clock>;

/**
 * @brief Now time
 *
 * @return Timestamp refering to the moment it is called
 */
DDSROUTER_UTILS_DllAPI Timestamp now() noexcept;

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERUTILS_TIME_HPP_ */