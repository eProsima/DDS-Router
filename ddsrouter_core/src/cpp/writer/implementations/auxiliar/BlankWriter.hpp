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
 * @file BlankWriter.hpp
 */

#ifndef __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_AUXILIAR_VOIDWRITER_HPP_
#define __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_AUXILIAR_VOIDWRITER_HPP_

#include <writer/IWriter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Writer that has an empty implementation.
 * It does not send anything.
 */
class BlankWriter : public IWriter
{
public:

    //! Override enable() IWriter method
    void enable() noexcept override;

    //! Override disable() IWriter method
    void disable() noexcept override;

    //! Override write() IWriter method
    utils::ReturnCode write(
            fastrtps::rtps::CacheChange_t* reader_cache_change) noexcept override;
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_AUXILIAR_VOIDWRITER_HPP_ */
