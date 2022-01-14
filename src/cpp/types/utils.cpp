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
 * @file utils.cpp
 *
 */

#include <algorithm>
#include <assert.h>
#include <set>

#if defined(_WIN32)
#include "Shlwapi.h"
#else
#include <fnmatch.h>
#endif // if defined(_WIN32)

#include <ddsrouter/exceptions/ValueNotAllowedException.hpp>
#include <ddsrouter/types/utils.hpp>
#include <ddsrouter/types/Log.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

bool match_pattern(
        const std::string& pattern,
        const std::string& str) noexcept
{
#if defined(_WIN32)
    // Windows implementation
    return PathMatchSpec(str.c_str(), pattern.c_str());
#else
    // Posix implementation
    return (fnmatch(pattern.c_str(), str.c_str(), FNM_NOESCAPE) == 0);
#endif // defined(_WIN32)
}

void to_lowercase(
        std::string& st) noexcept
{
    std::transform(st.begin(), st.end(), st.begin(),
            [](unsigned char c)
            {
                return std::tolower(c);
            });
}

std::string Formatter::to_string() const noexcept
{
    return ss_.str().c_str();
}

void tsnh(
        const Formatter& formatter)
{
    logError(DDSROUTER_TSNH, "This Should Have Not Happened: " << formatter.to_string());
    assert(false);
    throw ValueNotAllowedException(formatter);
}

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */
