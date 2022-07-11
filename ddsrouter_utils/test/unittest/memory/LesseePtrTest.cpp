// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <condition_variable>
#include <cstring>
#include <string>
#include <thread>

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddsrouter_utils/memory/owner_ptr.hpp>

using namespace eprosima::ddsrouter::utils;

/**
 * Create an owner and a lease from it.
 * Check once the owner has reset the value, the lease cannot lock the data (the original value has been destroyed)
 */
TEST(LesseePtrTest, lessee_ptr_string_reset)
{
    const char* internal_str_1 = "StringTest1";
    const char* internal_str_2 = "StringTest2";

    // Create Owner
    OwnerPtr<std::string> owner(new std::string(internal_str_1));

    // Create Lessee 1
    LesseePtr<std::string> lessee = owner.lease();
    ASSERT_EQ(0, std::strcmp(lessee.lock()->c_str(), internal_str_1));

    // Destroy element
    owner.reset(new std::string(internal_str_2));

    // Try to access lessee by lock and by operator->
    ASSERT_FALSE(lessee.lock());
}

/**
 * Access a value inside a \c LesseePtr class
 */
TEST(LesseePtrTest, lessee_ptr_string_access)
{
    const char* internal_str = "StringTest";
    OwnerPtr<std::string> owner(new std::string(internal_str));

    // Create Lessee
    LesseePtr<std::string> lessee = owner.lease();

    // Access by lock
    {
        auto str_ptr = lessee.lock();
        ASSERT_EQ(0, std::strcmp(str_ptr->c_str(), internal_str));
    }
}

/**
 * Access to a value inside a \c LesseePtr class from various points
 */
TEST(LesseePtrTest, lessee_ptr_string_multiple_access)
{
    const char* internal_str = "StringTest";
    OwnerPtr<std::string> owner(new std::string(internal_str));

    // Create Lessee 1
    LesseePtr<std::string> lessee_1 = owner.lease();
    ASSERT_EQ(0, std::strcmp(lessee_1.lock()->c_str(), internal_str));

    // Create Lessee 2
    LesseePtr<std::string> lessee_2 = owner.lease();
    ASSERT_EQ(0, std::strcmp(lessee_2.lock()->c_str(), internal_str));
    ASSERT_EQ(0, std::strcmp(lessee_2.lock()->c_str(), lessee_1.lock()->c_str()));

    // Accessing both concurrently
    {
        auto ptr_1 = lessee_1.lock();
        auto ptr_2 = lessee_2.lock();

        ASSERT_EQ(0, std::strcmp(ptr_1->c_str(), ptr_2->c_str()));
    }
}

/**
 * Create a lease, and acces its value after destroying the value in OwnerPtr
 */
TEST(LesseePtrTest, lessee_ptr_string_access_after_destroy)
{
    const char* internal_str = "StringTest";

    // Create Owner
    OwnerPtr<std::string> owner(new std::string(internal_str));

    // Create Lessee 1
    LesseePtr<std::string> lessee = owner.lease();
    ASSERT_EQ(0, std::strcmp(lessee.lock()->c_str(), internal_str));

    // Destroy element
    owner.reset();

    // Try to access lessee by lock and by operator->
    ASSERT_FALSE(lessee.lock());
}

/**
 * Check that while a lessee is blocked, the owner cannot be destroyed, and must wait for lessee to destroy element
 *
 * This is the thread schema. A condition variable and a int will work for thread synchronization:
 *
 *                                  1
 * ---TEST_THREAD---------------------try_to_destroy-----------------try_create_lease
 *                  ---create_lease---sleep------------release_lease---
 */
TEST(LesseePtrTest, lessee_ptr_string_access_lock_before_destroy)
{
    // Create a lease and access it by lock, then trying to remove the owner should wait until the lock is released
    const char* internal_str = "StringTest";

    std::mutex cv_mutex;
    std::condition_variable cv;
    int synchronization_step = 0;

    // Create Owner
    OwnerPtr<std::string> owner(new std::string(internal_str));

    // Create Lessee 1
    LesseePtr<std::string> lessee = owner.lease();

    // Execute thread with a reference locked
    std::thread lease_thread ([&lessee, &cv_mutex, &cv, &synchronization_step]()
            {
                // Create blocking reference
                auto reference_blocking_owner = lessee.lock();

                // Set step 1 is ready
                {
                    std::unique_lock<std::mutex> lock(cv_mutex);
                    synchronization_step = 1;
                    cv.notify_all();
                }

                // Sleep for a small time to let the owner arrive to delete
                std::this_thread::sleep_for(std::chrono::milliseconds(250));

                // At this point, the owner is waiting to be destroyed, but cant because this is locking it:
                reference_blocking_owner->append("_crazy_access_to_previous_deletion_memory"); // This would break if error

                // At this point the lease will be relased and the owner could be destroyed
            });

    // WAIT for step 1
    {
        std::unique_lock<std::mutex> lock(cv_mutex);
        cv.wait(lock, [&]()
                {
                    return synchronization_step == 1;
                });
    }

    // Destroy owner (this must wait until the lease is released)
    owner.reset();

    // Wait the thread is finished
    lease_thread.join();

    // Try to access again lessee by lock and should not have access
    ASSERT_FALSE(lessee.lock());
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
