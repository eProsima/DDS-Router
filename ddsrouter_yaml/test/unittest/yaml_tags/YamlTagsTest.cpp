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

#include <iostream>

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddsrouter_yaml/yaml_configuration_tags.hpp>

using namespace eprosima::ddsrouter::yaml;

TEST(YamlTagsTest, yaml_tags)
{
    std::set<std::string> retrieved_tags = ddsrouter_tags();

    std::set<std::string> current_tags =
    {
        ALLOWLIST_TAG,
        BLOCKLIST_TAG,
        TOPIC_NAME_TAG,
        TOPIC_TYPE_NAME_TAG,
        TOPIC_KIND_TAG,
        TOPIC_RELIABLE_TAG,
        PARTICIPANT_KIND_TAG,
        PARTICIPANT_NAME_TAG,
        COLLECTION_PARTICIPANTS_TAG,
        DOMAIN_ID_TAG,
        DISCOVERY_SERVER_GUID_PREFIX_TAG,
        LISTENING_ADDRESSES_TAG,
        CONNECTION_ADDRESSES_TAG,
        COLLECTION_ADDRESSES_TAG,
        TLS_TAG,
        TLS_CA_TAG,
        TLS_PASSWORD_TAG,
        TLS_PRIVATE_KEY_TAG,
        TLS_CERT_TAG,
        TLS_DHPARAMS_TAG,
        ADDRESS_IP_TAG,
        ADDRESS_DNS_TAG,
        ADDRESS_PORT_TAG,
        ADDRESS_IP_VERSION_TAG,
        ADDRESS_IP_VERSION_V4_TAG,
        ADDRESS_IP_VERSION_V6_TAG,
        ADDRESS_TRANSPORT_TAG,
        ADDRESS_TRANSPORT_UDP_TAG,
        ADDRESS_TRANSPORT_TCP_TAG,
        DISCOVERY_SERVER_GUID_TAG,
        DISCOVERY_SERVER_ID_TAG,
        DISCOVERY_SERVER_ID_ROS_TAG,
        VERSION_TAG,
        VERSION_TAG_V_1_0,
        VERSION_TAG_V_2_0,
        PARTICIPANT_KIND_TAG_V1,
    };

    ASSERT_EQ(retrieved_tags, current_tags);
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
