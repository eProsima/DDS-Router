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
 * @file utils.ipp
 */

#ifndef _DDSROUTERUTILS_FORMAT_IMPL_FORMATUTILS_IPP_
#define _DDSROUTERUTILS_FORMAT_IMPL_FORMATUTILS_IPP_

namespace eprosima {
namespace ddsrouter {
namespace utils {

template <typename T>
std::ostream& element_to_stream(
        std::ostream& os,
        const T& element)
{
    if (std::is_pointer<T>::value)
    {
        os << (*element);
    }
    else
    {
        os << element;
    }
    return os;
}

template <typename T>
std::ostream& container_to_stream(
        std::ostream& os,
        const std::vector<T>& list,
        const std::string& separator /* = ";"*/)
{
    if (!list.size() == 0)
    {
        typename std::vector<T>::const_iterator it = list.begin();
        element_to_stream<T>(os, *it);
        ++it;
        for (; it != list.end(); ++it)
        {
            os << separator;
            element_to_stream<T>(os, *it);
        }
    }

    return os;
}

template <typename T>
std::ostream& container_to_stream(
        std::ostream& os,
        const std::set<T>& set,
        const std::string& separator /* = ";" */)
{
    if (!set.size() == 0)
    {
        typename std::set<T>::const_iterator it = set.begin();
        element_to_stream<T>(os, *it);
        ++it;
        for (; it != set.end(); ++it)
        {
            os << separator;
            element_to_stream<T>(os, *it);
        }
    }

    return os;
}

template <typename T>
std::string to_string(
        const T& element) noexcept
{
    std::ostringstream os;
    element_to_stream<T>(os, element);
    return os.str();
}

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERUTILS_FORMAT_IMPL_FORMATUTILS_IPP_ */
