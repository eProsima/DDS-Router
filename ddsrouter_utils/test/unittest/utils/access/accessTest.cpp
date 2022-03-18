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

#include <ddsrouter_utils/utils.hpp>

using namespace eprosima::ddsrouter::utils;

/**
 * Check file exist
 */
TEST(accessTest, check_exist)
{
    // Default argument
    ASSERT_TRUE(is_file_accessible("test_files/r__.test"));
    ASSERT_FALSE(is_file_accessible("test_files/not_exist.test"));

    // Explicit argument
    ASSERT_TRUE(is_file_accessible("test_files/r__.test", EXIST));
    ASSERT_FALSE(is_file_accessible("test_files/not_exist.test", EXIST));
}

/**
 * Check file read permission
 */
TEST(accessTest, check_r)
{
    ASSERT_TRUE(is_file_accessible("test_files/r__.test", READ));
    ASSERT_TRUE(is_file_accessible("test_files/rw_.test", READ));
    ASSERT_TRUE(is_file_accessible("test_files/r_x.test", READ));
    ASSERT_TRUE(is_file_accessible("test_files/rwx.test", READ));

    ASSERT_FALSE(is_file_accessible("test_files/not_exist.test", READ));

    // WARNING: This cannot be tested in non readable cases because
    // it requires to copy a file without read permissions in test build, which cannot be done
    // as far as we know
    // ASSERT_FALSE(is_file_accessible("test_files/exist.test", READ));
}

/**
 * Check file write permission
 */
TEST(accessTest, check_w)
{
    ASSERT_TRUE(is_file_accessible("test_files/rw_.test", WRITE));
    ASSERT_TRUE(is_file_accessible("test_files/rwx.test", WRITE));

    ASSERT_FALSE(is_file_accessible("test_files/r__.test", WRITE));
    ASSERT_FALSE(is_file_accessible("test_files/r_x.test", WRITE));
    ASSERT_FALSE(is_file_accessible("test_files/not_exist.test", WRITE));
}

/**
 * Check file read permission
 */
TEST(accessTest, check_x)
{
    ASSERT_TRUE(is_file_accessible("test_files/r_x.test", EXECUTION));
    ASSERT_TRUE(is_file_accessible("test_files/rwx.test", EXECUTION));

    ASSERT_FALSE(is_file_accessible("test_files/r__.test", EXECUTION));
    ASSERT_FALSE(is_file_accessible("test_files/rw_.test", EXECUTION));
    ASSERT_FALSE(is_file_accessible("test_files/not_exist.test", EXECUTION));
}

/**
 * Check file read, write, and execution permission
 */
TEST(accessTest, check_rwx)
{
    ASSERT_TRUE(is_file_accessible("test_files/rwx.test", READ | WRITE | EXECUTION));

    ASSERT_FALSE(is_file_accessible("test_files/r__.test", READ | WRITE | EXECUTION));
    ASSERT_FALSE(is_file_accessible("test_files/rw_.test", READ | WRITE | EXECUTION));
    ASSERT_FALSE(is_file_accessible("test_files/r_x.test", READ | WRITE | EXECUTION));
    ASSERT_FALSE(is_file_accessible("test_files/not_exist.test", READ | WRITE | EXECUTION));
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
