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
 * @file IWriter.hpp
 */

#ifndef _DDSROUTER_WRITER_IDDS_ROUTERWRITER_HPP_
#define _DDSROUTER_WRITER_IDDS_ROUTERWRITER_HPP_

#include <ddsrouter/types/Data.hpp>
#include <ddsrouter/types/ReturnCode.hpp>
#include <ddsrouter/types/topic/RealTopic.hpp>
#include <ddsrouter/communication/PayloadPool.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * TODO
 */
class IWriter
{
public:

    IWriter(
            RealTopic,
            std::shared_ptr<PayloadPool>);

    virtual void enable();

    virtual void disable();

    virtual ReturnCode write(
            std::unique_ptr<DataReceived>&);
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_WRITER_IDDS_ROUTERWRITER_HPP_ */
