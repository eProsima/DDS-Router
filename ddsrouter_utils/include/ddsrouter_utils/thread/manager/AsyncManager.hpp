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

/**
 * @file AsyncManager.hpp
 *
 * This file contains class AsyncManager definition.
 */

#pragma once

#include <map>
#include <mutex>
#include <thread>
#include <vector>

#include <ddsrouter_utils/library/library_dll.h>
#include <ddsrouter_utils/thread/manager/IManager.hpp>
#include <ddsrouter_utils/thread/thread/CustomThread.hpp>
#include <ddsrouter_utils/types/Atomicable.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {
namespace thread {

using TasksCollectionType =
    Atomicable<
        std::vector<
            std::pair<
                std::unique_ptr<CustomThread>,
                std::unique_ptr<ITask>>>>;

/**
 * TODO
 */
class AsyncManager : public IManager
{
public:

    AsyncManager() = default;

    ~AsyncManager();

    virtual void execute(std::unique_ptr<ITask>&& task) override;

    void clean_threads();

protected:

    TasksCollectionType tasks_running_;
};

} /* namespace thread */
} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */
