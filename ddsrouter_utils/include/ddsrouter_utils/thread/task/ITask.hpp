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
 * @file ITask.hpp
 *
 * This file contains class Task definition.
 */

#ifndef _DDSROUTERTHREAD_THREAD_TASK_ITASK_HPP_
#define _DDSROUTERTHREAD_THREAD_TASK_ITASK_HPP_

namespace eprosima {
namespace ddsrouter {
namespace utils {
namespace thread {

class ITask
{
public:
    virtual ~ITask() {};
    virtual void operator()() noexcept = 0;
};

} /* namespace thread */
} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERTHREAD_THREAD_TASK_ITASK_HPP_ */
