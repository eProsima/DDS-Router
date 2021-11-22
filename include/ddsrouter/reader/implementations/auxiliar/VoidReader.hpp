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
 * @file VoidReader.hpp
 */

#ifndef _DDSROUTER_READER_IMPLEMENTATIONS_AUX_VOIDREADER_HPP_
#define _DDSROUTER_READER_IMPLEMENTATIONS_AUX_VOIDREADER_HPP_

#include <ddsrouter/reader/IReader.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * Reader that has an empty implementation.
 * It does not receive anything.
 * It does not have messages to take.
 */
class VoidReader : public IReader
{
public:

    //! Override enable() IReader method
    void enable() noexcept override;

    //! Override disable() IReader method
    void disable() noexcept override;

    //! Override set_on_data_available_callback() IReader method
    void set_on_data_available_callback(
            std::function<void()> on_data_available_lambda) noexcept override;

    //! Override unset_on_data_available_callback() IReader method
    void unset_on_data_available_callback() noexcept override;

    //! Override take() IReader method
    ReturnCode take(
            std::unique_ptr<DataReceived>& data) noexcept override;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_READER_IMPLEMENTATIONS_AUX_VOIDREADER_HPP_ */
