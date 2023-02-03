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
 * @file IRoutingData.hpp
 */

#ifndef _DDSROUTERCORE_TYPES_DDS_DATA_HPP_
#define _DDSROUTERCORE_TYPES_DDS_DATA_HPP_

#include <ddsrouter_core/library/library_dll.h>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * @brief TODO
 */
class IRoutingData
{

public:

    /**
     * @brief Virtual dtor.
     *
     * @note Default destructor. Force \c DataReceived to be polymorphic. Implemented here to avoid creating a .cpp .
     */
    virtual ~IRoutingData()
    {
    }
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_DDS_DATA_HPP_ */
