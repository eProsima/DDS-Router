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

#include <assert.h>

#include <ddsrouter_utils/math/math.hpp>

bool is_even(
        uint32_t number) noexcept
{
    return (number & 0x1) == 0;
}

bool is_power_of_2(
        uint32_t number) noexcept
{
    return number && (!(number & (number-1)));
}

uint32_t fast_module(
        uint32_t dividend,
        uint32_t divisor) noexcept
{
    assert(divisor != 0);

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
    else if (is_power_of_2(divisor))
    {
        // Optimize to ~6 operations [if, if, if, if(and), and]
        return dividend & (divisor - 1);
    }
    else
    {
        // Not optimum
        return dividend % divisor;
    }
}

uint32_t fast_division(
        uint32_t dividend,
        uint32_t divisor) noexcept
{
    assert(divisor != 0);

    if (dividend < divisor)
    {
        // Optimize to 1 operation [if]
        return 0;
    }
    else if (dividend == divisor)
    {
        // Optimize to 2 operation [if, if]
        return 1;
    }
    else if (divisor == 1)
    {
        // Optimize to 3 operations [if, if, if]
        return dividend;
    }
    else if (divisor == 2)
    {
        // Optimize to 5 operations [if, if, if, if, swift]
        return (dividend >> 1);
    }
    else if (is_power_of_2(divisor))
    {
        while (divisor != 1)
        {
            dividend >>= 1;
            divisor >>= 1;
        }
        return dividend;
    }
    else
    {
        // Not optimum
        return dividend / divisor;
    }
}

uint32_t arithmetic_progression_sum(
        uint32_t lowest,
        uint32_t interval,
        uint32_t steps) noexcept
{
    return (((2 * lowest + ((steps - 1) * interval)) * steps) / 2);
}

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */
