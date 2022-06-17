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
 * @file math.cpp
 *
 */

namespace eprosima {
namespace ddsrouter {
namespace utils {

#include <ddsrouter_utils/math/math.hpp>

uint32_t fast_module(
        uint32_t dividend,
        uint32_t divisor) noexcept
{
    if (dividend < divisor)
    {
        // Optimize to 1 operation [if]
        return dividend;
    }
    else if (dividend == divisor)
    {
        // Optimize to 2 operation [if, if]
        return 0;
    }
    else if (divisor == 2)
    {
        // Optimize to 4 operations [if, if, if, and]
        return dividend & 1;
    }
    else
    {
        // Optimize to 7 operations [if, if, if, -, and, -, and] in case E(n){divisor = 2^n}
        return divisor & (divisor - 1) ? dividend % divisor : dividend& (divisor - 1);
    }
}

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */
