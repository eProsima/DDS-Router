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
 * @file Task.hpp
 *
 * This file contains class Task definition.
 */

#ifndef _DDSROUTERTHREAD_TASK_TASK_HPP_
#define _DDSROUTERTHREAD_TASK_TASK_HPP_

#include <functional>

namespace eprosima {
namespace ddsrouter {
namespace utils {

/**
 * This class represents a task that can be executed by a Thread Pool.
 *
 * @note this first implementation only uses this class as a \c std::function<void()> for simplicity.
 * In future implementations, this could be a more complex class.
 */
class Task : public std::function<void()>
{
    using std::function<void()>::function;
};

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERTHREAD_TASK_TASK_HPP_ */
