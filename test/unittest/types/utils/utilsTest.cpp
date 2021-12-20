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

#include <ddsrouter/types/utils.hpp>

using namespace eprosima::ddsrouter::utils;

/**
 * Test \c set_of_ptr_contains method for a set of pointers to integers
 *
 * Cases:
 *  empty set
 *  element with same ptr contained
 *  element contained
 *  element not contained
 *  nullptr not contained
 */
TEST(utilsTest, set_of_ptr_contains_int)
{
    // Create an empty set
    std::set<std::shared_ptr<int>> set;

    // Elements to add
    std::shared_ptr<int> ptr0 = std::make_shared<int>(0);
    std::shared_ptr<int> ptr1 = std::make_shared<int>(1);
    std::shared_ptr<int> ptr2 = std::make_shared<int>(2);
    std::shared_ptr<int> ptr3 = std::make_shared<int>(3);

    // Elements to look for
    std::shared_ptr<int> ptr_0b = ptr0; // same ptr
    std::shared_ptr<int> ptr_1b = std::make_shared<int>(1); // different ptr
    std::shared_ptr<int> ptr_4 = std::make_shared<int>(4);
    std::shared_ptr<int> ptr_null; // nullptr

    {
        // empty set
        ASSERT_FALSE(set_of_ptr_contains(set, ptr_0b));
        ASSERT_FALSE(set_of_ptr_contains(set, ptr_1b));
        ASSERT_FALSE(set_of_ptr_contains(set, ptr_4));
        ASSERT_FALSE(set_of_ptr_contains(set, ptr_null));
    }

    // Add elements to set
    set.insert(ptr0);
    set.insert(ptr1);
    set.insert(ptr2);
    set.insert(ptr3);

    {
        // element with same ptr contained
        ASSERT_EQ(ptr0, ptr_0b);
        ASSERT_TRUE(set_of_ptr_contains(set, ptr_0b));
    }

    {
        // element contained
        ASSERT_NE(ptr1, ptr_1b);
        ASSERT_TRUE(set_of_ptr_contains(set, ptr_1b));
    }

    {
        // element not contained
        ASSERT_FALSE(set_of_ptr_contains(set, ptr_4));
    }

    {
        // nullptr not contained
        ASSERT_FALSE(set_of_ptr_contains(set, ptr_null));
    }
}

/**
 * Test \c set_of_ptr_contains method for a set of pointers to integers that contains a nullptr
 *
 * Cases:
 *  nullptr contained
 *  element contained
 *  element not contained
 */
TEST(utilsTest, set_of_ptr_with_null_contains_int)
{
    // Create an empty set
    std::set<std::shared_ptr<int>> set;

    // Elements to add
    std::shared_ptr<int> ptr0 = std::make_shared<int>(0);
    std::shared_ptr<int> ptr1 = std::make_shared<int>(1);
    std::shared_ptr<int> ptr2 = std::make_shared<int>(2);
    std::shared_ptr<int> ptr3 = std::make_shared<int>(3);
    std::shared_ptr<int> ptrn; // nullptr

    // Elements to look for
    std::shared_ptr<int> ptr_0b = ptr0; // same ptr
    std::shared_ptr<int> ptr_1b = std::make_shared<int>(1); // different ptr
    std::shared_ptr<int> ptr_4 = std::make_shared<int>(4);
    std::shared_ptr<int> ptr_null_b = ptrn; // same direction as ptrn
    std::shared_ptr<int> ptr_null; // nullptr

    // Add elements
    set.insert(ptr0);
    set.insert(ptr1);
    set.insert(ptr2);
    set.insert(ptr3);
    set.insert(ptrn);

    {
        // nullptr contained
        ASSERT_TRUE(set_of_ptr_contains(set, ptr_null_b));
        ASSERT_TRUE(set_of_ptr_contains(set, ptr_null));
    }

    {
        // element contained
        ASSERT_TRUE(set_of_ptr_contains(set, ptr_0b));
        ASSERT_TRUE(set_of_ptr_contains(set, ptr_1b));
    }

    {
        // element not contained
        ASSERT_FALSE(set_of_ptr_contains(set, ptr_4));
    }
}

/**
 * Test \c set_of_ptr_contains method for a set of pointers to string
 *
 * Cases:
 *  element contained
 *  element not contained
 */
TEST(utilsTest, set_of_ptr_contains_string)
{
    // Create an empty set
    std::set<std::shared_ptr<std::string>> set;

    // Elements to add
    std::shared_ptr<std::string> ptr_a = std::make_shared<std::string>("a");
    std::shared_ptr<std::string> ptr_barro = std::make_shared<std::string>("Barro");
    std::shared_ptr<std::string> ptr_with_spaces = std::make_shared<std::string>("String with spaces");
    std::shared_ptr<std::string> ptr_very_long_st = std::make_shared<std::string>(
        "I count him braver who overcomes his desires than him who conquers his enemies, "
        "for the hardest victory is over self. ― Aristotle"
    );

    // Add elements to set
    set.insert(ptr_a);
    set.insert(ptr_barro);
    set.insert(ptr_with_spaces);
    set.insert(ptr_very_long_st);

    // Elements to look for
    std::shared_ptr<std::string> ptr_ab = std::make_shared<std::string>("a");; // contained
    std::shared_ptr<std::string> ptr_b = std::make_shared<std::string>("Barro");; // contained
    std::shared_ptr<std::string> ptr_b2 = std::make_shared<std::string>("Barr");; // substring contained
    std::shared_ptr<std::string> ptr_spaces_b = std::make_shared<std::string>("Stringwithspaces"); // almost contained
    std::shared_ptr<std::string> ptr_4 = std::make_shared<std::string>("4"); // not contained

    {
        // element contained
        ASSERT_TRUE(set_of_ptr_contains(set, ptr_ab));
        ASSERT_TRUE(set_of_ptr_contains(set, ptr_b));
    }

    {
        // element not contained
        ASSERT_FALSE(set_of_ptr_contains(set, ptr_spaces_b));
        ASSERT_FALSE(set_of_ptr_contains(set, ptr_b2));
        ASSERT_FALSE(set_of_ptr_contains(set, ptr_4));
    }
}

/**
 * Test \c are_set_of_ptr_equal method for a set of pointers to integers
 *
 * Each case is tested in both directions to check commutativity
 *
 * Cases:
 *  both empty set
 *  one empty set
 *  same elements
 *  different elements different size
 *  different elements same size
 *  different elements one with nullptr
 *  same elements with nullptr
 *  each with itself
 */
TEST(utilsTest, are_set_of_ptr_equal_int)
{
    // Create an empty set
    std::set<std::shared_ptr<int>> set1;
    std::set<std::shared_ptr<int>> set2;

    // Elements to add to set 1
    std::shared_ptr<int> ptra_0 = std::make_shared<int>(0);
    std::shared_ptr<int> ptra_1 = std::make_shared<int>(1);
    std::shared_ptr<int> ptra_2 = std::make_shared<int>(2);
    std::shared_ptr<int> ptra_3 = std::make_shared<int>(3);
    std::shared_ptr<int> ptra_4 = std::make_shared<int>(4);
    std::shared_ptr<int> ptra_n; // nullptr

    // Elements to add to set 2
    std::shared_ptr<int> ptrb_0 = ptra_0; // same object
    std::shared_ptr<int> ptrb_1 = std::make_shared<int>(1);
    std::shared_ptr<int> ptrb_2 = std::make_shared<int>(2);
    std::shared_ptr<int> ptrb_3 = std::make_shared<int>(3);
    std::shared_ptr<int> ptrb_4 = std::make_shared<int>(4);
    std::shared_ptr<int> ptrb_n; // nullptr

    {
        // both empty set
        ASSERT_TRUE(are_set_of_ptr_equal(set1, set2));
        ASSERT_EQ(are_set_of_ptr_equal(set1, set2), are_set_of_ptr_equal(set2, set1));
    }

    // Add elements to set 1
    set1.insert(ptra_0);
    set1.insert(ptra_1);
    set1.insert(ptra_2);

    {
        // one empty set
        ASSERT_FALSE(are_set_of_ptr_equal(set1, set2));
        ASSERT_EQ(are_set_of_ptr_equal(set1, set2), are_set_of_ptr_equal(set2, set1));
    }

    // Add elements to set 2
    set2.insert(ptrb_0);
    set2.insert(ptrb_1);
    set2.insert(ptrb_2);

    {
        // same elements
        ASSERT_TRUE(are_set_of_ptr_equal(set1, set2));
        ASSERT_EQ(are_set_of_ptr_equal(set1, set2), are_set_of_ptr_equal(set2, set1));
    }

    // Add elements to set 1
    set1.insert(ptra_3);

    {
        // different elements different size
        ASSERT_FALSE(are_set_of_ptr_equal(set1, set2));
        ASSERT_EQ(are_set_of_ptr_equal(set1, set2), are_set_of_ptr_equal(set2, set1));
    }

    // Add elements to set 2
    set2.insert(ptrb_4);

    {
        // different elements same size
        ASSERT_FALSE(are_set_of_ptr_equal(set1, set2));
        ASSERT_EQ(are_set_of_ptr_equal(set1, set2), are_set_of_ptr_equal(set2, set1));
    }

    // Add nullptr to set 1
    set1.insert(ptra_n);
    // Add 3 to set 2 so both has same size
    set2.insert(ptrb_3);

    {
        // different elements one with nullptr
        ASSERT_FALSE(are_set_of_ptr_equal(set1, set2));
        ASSERT_EQ(are_set_of_ptr_equal(set1, set2), are_set_of_ptr_equal(set2, set1));
    }

    // Add nullptr to set 2
    set2.insert(ptrb_n);
    // Add 4 to set 1 so both has same elements
    set1.insert(ptra_4);

    {
        // same elements with nullptr
        ASSERT_TRUE(are_set_of_ptr_equal(set1, set2));
        ASSERT_EQ(are_set_of_ptr_equal(set1, set2), are_set_of_ptr_equal(set2, set1));
    }

    {
        // each with itself
        ASSERT_TRUE(are_set_of_ptr_equal(set1, set1));
        ASSERT_TRUE(are_set_of_ptr_equal(set2, set2));
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
