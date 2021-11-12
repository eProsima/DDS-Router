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
 * @file IDatabrokerReader.hpp
 */

#ifndef _DATABROKER_READER_IDATABROKERREADER_HPP_
#define _DATABROKER_READER_IDATABROKERREADER_HPP_

#include <functional>

#include <databroker/types/Data.hpp>
#include <databroker/types/ReturnCode.hpp>
#include <databroker/types/topic/RealTopic.hpp>
#include <databroker/communication/PayloadPool.hpp>

namespace eprosima {
namespace databroker {

/**
 * TODO
 */
class IDatabrokerReader
{
public:

    IDatabrokerReader(
            RealTopic,
            std::shared_ptr<PayloadPool>);

    virtual void enable();

    /**
     *
     * ATTENTION: This method should stop calling the callback \c on_data_available_lambda if more data arrives while
     * disabled.
     */
    virtual void disable();

    virtual ReturnCode set_on_data_available_callback(
            std::function<void()> on_data_available_lambda);

    virtual ReturnCode take(
            std::unique_ptr<DataReceived>&);         // This data read must be inside a PayloadPool
};

} /* namespace databroker */
} /* namespace eprosima */

#endif /* _DATABROKER_READER_IDATABROKERREADER_HPP_ */
