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

#include <algorithm>
#include <iostream>

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddsrouter/exceptions/ConfigurationException.hpp>
#include <ddsrouter/types/address/Address.hpp>
#include <ddsrouter/types/configuration_tags.hpp>
#include <ddsrouter/types/RawConfiguration.hpp>

using namespace eprosima::ddsrouter;

/**
 * Test AddressConfiguration port.
 *
 * CASES:
 *  Default case
 *  Specific port
 */
TEST(AddressConfigurationTest, yaml_port)
{
    // Default case
    {
        Address address = Address(RawConfiguration());
        ASSERT_EQ(address.port(), Address::default_port());
    }

    // Specific port
    {
        std::vector<PortType> ports = {1, 11, 111, 1111, 30000};

        for (PortType port : ports)
        {
            RawConfiguration yaml;
            yaml[ADDRESS_PORT_TAG] = port;

            Address address = Address(yaml);
            ASSERT_EQ(address.port(), port);
        }
    }
}

/**
 * Test AddressConfiguration Transport protocol.
 *
 * CASES:
 *  Default case
 *  Change Default case
 *  Specific transport protocol
 */
TEST(AddressConfigurationTest, yaml_transport_protocol)
{
    // Default case
    {
        Address address = Address(RawConfiguration());
        ASSERT_EQ(address.transport_protocol(), Address::default_transport_protocol());
    }

    // Change Default case
    {
        std::vector<TransportProtocol> tps = {UDP, TCP};

        for (TransportProtocol tp : tps)
        {
            Address address = Address(RawConfiguration(), tp);
            ASSERT_EQ(address.transport_protocol(), tp);
        }
    }

    // Specific transport protocol
    {
        std::vector<std::pair<TransportProtocol, const char*>> tps = {
            std::make_pair(UDP, ADDRESS_TRANSPORT_UDP_TAG),
            std::make_pair(TCP, ADDRESS_TRANSPORT_TCP_TAG)};

        for (std::pair<TransportProtocol, const char*> tp : tps)
        {
            RawConfiguration yaml;
            yaml[ADDRESS_TRANSPORT_TAG] = tp.second;

            Address address = Address(yaml);
            ASSERT_EQ(address.transport_protocol(), tp.first);
        }
    }
}

/**
 * Test AddressConfiguration IP v4.
 *
 * CASES:
 *  Default case
 *  Set ip with format IPv4
 *  Set IPv4 type
 */
TEST(AddressConfigurationTest, yaml_ipv4)
{
    // Default case
    {
        Address address = Address(RawConfiguration());
        ASSERT_EQ(address.ip(), Address::default_ip());
        ASSERT_EQ(address.ip_version(), Address::default_ip_version());
    }

    // Set ip with format IPv4
    {
        std::vector<IpType> ips = {
            "127.0.0.1",
            "8.8.8.8",
            "192.168.1.23"
        };

        for (IpType ip : ips)
        {
            RawConfiguration yaml;
            yaml[ADDRESS_IP_TAG] = ip;

            Address address = Address(yaml);
            ASSERT_EQ(address.ip(), ip);
            ASSERT_EQ(address.ip_version(), IPv4);
        }
    }

    // Set IPv4 type
    {
        std::vector<IpType> ips = {
            "127.0.0.1",
            "1.1.1.1"
        };

        for (IpType ip : ips)
        {
            RawConfiguration yaml;
            yaml[ADDRESS_IP_TAG] = ip;
            yaml[ADDRESS_IP_VERSION_TAG] = ADDRESS_IP_VERSION_V4_TAG;

            Address address = Address(yaml);
            ASSERT_EQ(address.ip(), ip);
            ASSERT_EQ(address.ip_version(), IPv4);
        }
    }
}

/**
 * Test AddressConfiguration IP v6.
 *
 * CASES:
 *  Set ip with format IPv6
 *  Set IPv6 type
 */
TEST(AddressConfigurationTest, yaml_ipv6)
{

    // Set ip with format IPv6
    {
        std::vector<IpType> ips = {
            "::1",
            "8::8",
            "2001:0DB8:0000:0000:0000:0000:1428:57ab"
        };

        for (IpType ip : ips)
        {
            RawConfiguration yaml;
            yaml[ADDRESS_IP_TAG] = ip;

            Address address = Address(yaml);
            ASSERT_EQ(address.ip(), ip);
            ASSERT_EQ(address.ip_version(), IPv6);
        }
    }

    // Set IPv6 type
    {
        std::vector<IpType> ips = {
            "::1",
            "::8:8:8:8"
        };

        for (IpType ip : ips)
        {
            RawConfiguration yaml;
            yaml[ADDRESS_IP_TAG] = ip;
            yaml[ADDRESS_IP_VERSION_TAG] = ADDRESS_IP_VERSION_V6_TAG;

            Address address = Address(yaml);
            ASSERT_EQ(address.ip(), ip);
            ASSERT_EQ(address.ip_version(), IPv6);
        }
    }
}

/**
 * Test AddressConfiguration port.
 *
 * Only test localhost to check the call is done correctly.
 * Adding other ips will lead to add DNS entries in the CI.
 * This is tested in Fast DDS <test/unittest/utils/LocatorTests.cpp> so not replicating here.
 *
 * CASES:
 *  Specific DNS addresses with default type
 *  Specific DNS addresses IPv4
 *  Specific DNS addresses IPv6 (not available)
 */
TEST(AddressConfigurationTest, yaml_dns)
{
    // Specific DNS addresses with default type
    {
        IpType dns_name = "localhost";
        IpType expected_address = "127.0.0.1";

        RawConfiguration yaml;
        yaml[ADDRESS_DNS_TAG] = dns_name;

        Address address = Address(yaml);
        ASSERT_EQ(address.ip(), expected_address);
        ASSERT_EQ(address.ip_version(), Address::default_ip_version());
    }

    // Specific DNS addresses IPv4
    {
        IpType dns_name = "localhost";
        IpType expected_address = "127.0.0.1";

        RawConfiguration yaml;
        yaml[ADDRESS_DNS_TAG] = dns_name;
        yaml[ADDRESS_IP_VERSION_TAG] = ADDRESS_IP_VERSION_V4_TAG;

        Address address = Address(yaml);
        ASSERT_EQ(address.ip(), expected_address);
        ASSERT_EQ(address.ip_version(), IPv4);
    }

    // Specific DNS addresses IPv6
    // This test could fail, as not every local DNS recognize ::1 as localhost IPv6
    // {
    //     IpType dns_name = "localhost";
    //     IpType expected_address = "::1";

    //     RawConfiguration yaml;
    //     yaml[ADDRESS_IP_TAG] = dns_name;
    //     yaml[ADDRESS_IP_VERSION_TAG] = ADDRESS_IP_VERSION_V6_TAG;

    //     Address address = Address(yaml);
    //     ASSERT_EQ(address.ip(), expected_address);
    //     ASSERT_EQ(address.ip_version(), IPv6);
    // }
}

/**
 * Test AddressConfiguration port.
 *
 * CASES:
 *  Scalar as yaml
 *  Array as yaml
 *  DNS fail to find network
 *  IP version error tag
 *  Transport protocol error tag
 */
TEST(AddressConfigurationTest, yaml_error_cases)
{
    // Scalar as yaml
    {
        RawConfiguration scalar_config;
        scalar_config = 666;

        ASSERT_THROW(new Address(scalar_config), ConfigurationException);
    }

    // Array as yaml
    {
        RawConfiguration array_config;
        array_config.push_back("rand_val_1");
        array_config.push_back("rand_val_2");

        ASSERT_THROW(new Address(array_config), ConfigurationException);
    }

    // DNS fail to find network
    {
        RawConfiguration yaml;
        yaml[ADDRESS_IP_TAG] = "thishostdoesnotexist_andmustnotexist.zzz";

        ASSERT_THROW(new Address(yaml), ConfigurationException);
    }

    // IP version error tag
    {
        RawConfiguration yaml;
        yaml[ADDRESS_IP_VERSION_TAG] = "IPv5";

        ASSERT_THROW(new Address(yaml), ConfigurationException);
    }

    // Transport protocol error tag
    {
        RawConfiguration yaml;
        yaml[ADDRESS_TRANSPORT_TAG] = "UTP";

        ASSERT_THROW(new Address(yaml), ConfigurationException);
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
