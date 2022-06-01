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
 * @file TaskId.cpp
 *
 * This file contains class TaskId implementation.
 */

#include <random>

#include <ddsrouter_thread/task/TaskId.hpp>

namespace eprosima {
namespace ddsrouter {
namespace thread {

TaskId new_unique_task_id()
{
    return static_cast<TaskId>(std::rand());
}

} /* namespace thread */
} /* namespace ddsrouter */
} /* namespace eprosima */
