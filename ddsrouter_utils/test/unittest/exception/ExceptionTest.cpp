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

#include <algorithm>

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddsrouter_utils/exception/Exception.hpp>

using namespace eprosima::ddsrouter::utils;

/**
 * Test \c Exception constructor from string method
 */
TEST(ExceptionTest, string_constructor)
{
    std::vector<std::string> cases({
        "This is an exception created from string.",
        "This is another exception.",
        "And one last one from string."
    });

    for (std::string exception_str : cases)
    {
        Exception __useless_exception(exception_str);
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
