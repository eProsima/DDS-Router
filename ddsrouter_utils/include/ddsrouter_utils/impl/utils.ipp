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

#ifndef _DDSROUTERUTILS_IMPL_UTILS_IPP_
#define _DDSROUTERUTILS_IMPL_UTILS_IPP_

namespace eprosima {
namespace ddsrouter {
namespace utils {

template <typename T, bool Ptr /* = false */>
std::ostream& element_to_stream(
        std::ostream& os,
        const T& element)
{
    if (Ptr)
    {
        // If pointer, print object and not address
        os << (*element);
    }
    else
    {
        os << element;
    }

    return os;
}

template <typename T, bool Ptr /* = false */>
std::ostream& container_to_stream(
        std::ostream& os,
        const std::vector<T>& list,
        const std::string& separator /* = ";"*/)
{
    os << "{";

    size_t size = list.size();

    for (size_t i = 0; size != 0 && i < size - 1; ++i)
    {
        element_to_stream<T, Ptr>(os, list[i]);
        os << separator;
    }

    // For the last element, to avoid separator
    if (size > 0)
    {
        element_to_stream<T, Ptr>(os, list[size - 1]);
    }

    os << "}";

    return os;
}

template <typename T, bool Ptr /* = false */>
std::ostream& container_to_stream(
        std::ostream& os,
        const std::set<T>& list,
        const std::string& separator /* = ";" */)
{
    return container_to_stream<T, Ptr>(os, std::vector<T>(list.begin(), list.end()), separator);
}

template <typename T>
bool set_of_ptr_contains(
        const std::set<std::shared_ptr<T>>& set,
        const std::shared_ptr<T>& element)
{
    // If the pointer belongs to set, it is contained
    if (set.find(element) != set.end())
    {
        return true;
    }

    // If element we are looking for is nullptr, do not search as we already know it is not in the set
    // (every nullptr is compared to equal)
    if (!element)
    {
        return false;
    }

    // If not, check if any object internally is the one we are looking for
    for (auto itr = set.begin(); itr != set.end(); itr++)
    {
        // In case the set element is nullptr, do not call == or it will crash
        if (nullptr != *itr)
        {
            if (*itr->get() == *element.get())
            {
                // If are equal, element is contained
                return true;
            }
        }
    }

    return false;
}

template <typename T>
bool are_set_of_ptr_equal(
        const std::set<std::shared_ptr<T>>& set1,
        const std::set<std::shared_ptr<T>>& set2)
{
    if (set1.size() != set2.size())
    {
        return false;
    }

    // Check if every element in set1 is in set2
    for (auto itr1 = set1.begin(); itr1 != set1.end(); itr1++)
    {
        if (!set_of_ptr_contains(set2, *itr1))
        {
            return false;
        }
    }

    return true;
}

template <typename Parent, typename Child>
std::set<std::shared_ptr<Parent>> convert_set_to_shared(
        const std::set<Child>& set)
{
    FORCE_TEMPLATE_SUBCLASS(Parent, Child);

    std::set<std::shared_ptr<Parent>> result_set;
    for (Child element : set)
    {
        result_set.insert(std::make_shared<Child>(element));
    }
    return result_set;
}

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERUTILS_IMPL_UTILS_IPP_ */
