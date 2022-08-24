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
 * @file types.hpp
 *
 * Contains auxiliary data types.
 */

#ifndef _DDSROUTERCORE_TYPES_COMMON_TYPES_HPP_
#define _DDSROUTERCORE_TYPES_COMMON_TYPES_HPP_

#include <set>

#include <ddsrouter_utils/types/Atomicable.hpp>

#include <ddsrouter_core/library/library_dll.h>
#include <ddsrouter_core/types/dds/Guid.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

using GuidPrefixDataFilterType = utils::SharedAtomicable<std::set<types::GuidPrefix>>;


} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_COMMON_TYPES_HPP_ */
