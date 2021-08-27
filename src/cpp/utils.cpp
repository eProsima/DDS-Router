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

#include <databroker/utils.hpp>

namespace eprosima {
namespace databroker {
namespace utils {

void to_lowercase(
        std::string& input)
{
    std::for_each(input.begin(), input.end(), [](char& c)
            {
                c = ::tolower(c);
            });
}

bool split_string(
        std::string input,
        std::vector<std::string>& output,
        const std::string& separator /* = ";" */)
{
    while (input.size())
    {
        int index = input.find(separator);
        if (index != std::string::npos)
        {
            output.push_back(input.substr(0, index));
            input = input.substr(index + separator.size());
        }
        else
        {
            output.push_back(input);
            break;
        }
    }
    return true;
}

} /* namespace utils */
} /* namespace databroker */
} /* namespace eprosima */
