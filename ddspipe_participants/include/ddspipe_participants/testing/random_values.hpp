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
// limitations under the License\.

#pragma once

#include <ddspipe_participants/library/library_dll.h>
#include <ddspipe_participants/types/address/Address.hpp>
#include <ddspipe_participants/types/address/DiscoveryServerConnectionAddress.hpp>

namespace eprosima {
namespace ddspipe {
namespace participants {
namespace testing {

DDSPIPE_PARTICIPANTS_DllAPI types::Address random_address(
        unsigned int seed = 0);

DDSPIPE_PARTICIPANTS_DllAPI types::DiscoveryServerConnectionAddress random_connection_address(
        unsigned int seed = 0,
        unsigned int size = 1,
        bool ros = false);

} /* namespace testing */
} /* namespace participants */
} /* namespace ddspipe */
} /* namespace eprosima */
