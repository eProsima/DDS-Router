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

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddsrouter_event/wait/CounterWaitHandler.hpp>
#include <ddsrouter_utils/Log.hpp>
#include <ddsrouter_utils/Time.hpp>

#include <ddsrouter_thread/manager/ThreadPoolManager.hpp>

using namespace eprosima::ddsrouter::thread;

/**
 * TODO
 */
TEST(DummyTest, dummy)
{
    {
        ThreadPoolManager manager(1);

        eprosima::ddsrouter::event::CounterWaitHandler waiter(0);

        Task task(
            [&waiter]
            ()
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                ++waiter;
            });

        eprosima::ddsrouter::utils::Timer timer;

        manager.emit(std::move(task));

        waiter.wait_upper_bound_threshold(0);

        ASSERT_LE(timer.elapsed_ms(), 2000);
    }
}

TEST(DummyTest, dummy_multiple)
{
    {
        ThreadPoolManager manager(3);

        eprosima::ddsrouter::event::CounterWaitHandler waiter(0);

        eprosima::ddsrouter::utils::Timer timer;

        for (int i = 0; i < 6; i++)
        {
            manager.emit(
                [&waiter]
                ()
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    ++waiter;
                });
        }

        waiter.wait_upper_bound_threshold(5);

        ASSERT_LE(timer.elapsed_ms(), 3000);
    }
}

int main(
        int argc,
        char** argv)
{
    eprosima::ddsrouter::utils::Log::SetVerbosity(eprosima::ddsrouter::utils::Log::Kind::Info);

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
