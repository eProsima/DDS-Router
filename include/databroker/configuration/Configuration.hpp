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
 * @file DatabrokerConfiguration.hpp
 */

#ifndef _DATABROKER_CONFIGURATION_CONFIGURATION_HPP_
#define _DATABROKER_CONFIGURATION_CONFIGURATION_HPP_

#include <databroker/types/RawConfiguration.hpp>
#include <databroker/types/topic/AbstractTopic.hpp>
#include <databroker/types/ReturnCode.hpp>

namespace eprosima {
namespace databroker {

/**
 * TODO
 */
class IConfiguration
{
public:
    IConfiguration(const RawConfiguration&);

    // Read the Yaml and get the params required
    // Fail in case he configuration is not correct
    virtual ReturnCode load() = 0;

protected:

    RawConfiguration configuration_;
};

/**
 * TODO
 */
class AllowedTopicConfiguration : public IConfiguration
{
public:
    std::list<AbstractTopic> whitelist() const;
    std::list<AbstractTopic> blacklist() const;
};

// TODO: create new interfaces for common configurations
class ListeningAddressConfiguration
{
public:
    // TODO
};

} /* namespace databroker */
} /* namespace eprosima */

#endif /* _DATABROKER_CONFIGURATION_CONFIGURATION_HPP_ */
