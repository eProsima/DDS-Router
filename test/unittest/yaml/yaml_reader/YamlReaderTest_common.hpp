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

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddsrouter/yaml/YamlReader.hpp>

namespace eprosima {
namespace ddsrouter {
namespace yaml {
namespace test {

class MockYamlReader : public YamlReader
{
public:

    // Make protected methods from parent public

    using YamlReader::get_value_in_tag;

    using YamlReader::get;

    using YamlReader::get_scalar;

    using YamlReader::get_list;

    using YamlReader::get_enumeration;
};

} /* namespace test */
} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */
