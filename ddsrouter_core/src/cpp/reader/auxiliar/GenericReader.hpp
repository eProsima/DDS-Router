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
 * @file GenericReader.hpp
 */

#ifndef __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_AUXILIAR_GENERICREADER_HPP_
#define __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_AUXILIAR_GENERICREADER_HPP_

#include <reader/IReader.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Generic reader interface
 */
template <types::ParticipantKind PartKind>
class GenericReader : public IReader
{
public:

    //! Use super-class constructors
    using IReader::IReader;

    void take_and_forward() noexcept override;
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_AUXILIAR_GENERICREADER_HPP_ */

