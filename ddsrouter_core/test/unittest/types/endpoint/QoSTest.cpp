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

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddsrouter_core/types/dds/QoS.hpp>

using namespace eprosima::ddsrouter::core;
using namespace eprosima::ddsrouter::core::types;

/*
 * Arbitrary value for reliability kind when real value is not important
 */
ReliabilityKind default_reliability_kind()
{
    return ReliabilityKind::BEST_EFFORT;
}

/*
 * Arbitrary value for durability kind when real value is not important
 */
DurabilityKind default_durability_kind()
{
    return DurabilityKind::VOLATILE;
}

/**
 * Test \c QoS constructor using Fast DDS QoS values
 */
TEST(QoSTest, constructor)
{
    DurabilityKind durability = default_durability_kind();
    ReliabilityKind reliability = default_reliability_kind();

    QoS qos(durability, reliability);

    ASSERT_EQ(qos.durability(), durability);
    ASSERT_EQ(qos.reliability(), reliability);
}

/**
 * Test \c QoS \c durability getter method
 *
 * CASES:
 *  Volatile
 *  TransientLocal
 *  Transient
 *  Persistent
 */
TEST(QoSTest, durability_getter)
{
    // Volatile
    {
        DurabilityKind durability = eprosima::fastrtps::rtps::DurabilityKind_t::VOLATILE;
        ReliabilityKind reliability = default_reliability_kind();

        QoS qos(durability, reliability);

        ASSERT_EQ(qos.durability(), eprosima::fastrtps::rtps::DurabilityKind_t::VOLATILE);
    }

    // TransientLocal
    {
        DurabilityKind durability = eprosima::fastrtps::rtps::DurabilityKind_t::TRANSIENT_LOCAL;
        ReliabilityKind reliability = default_reliability_kind();

        QoS qos(durability, reliability);

        ASSERT_EQ(qos.durability(), eprosima::fastrtps::rtps::DurabilityKind_t::TRANSIENT_LOCAL);
    }

    // Transient
    {
        DurabilityKind durability = eprosima::fastrtps::rtps::DurabilityKind_t::TRANSIENT;
        ReliabilityKind reliability = default_reliability_kind();

        QoS qos(durability, reliability);

        ASSERT_EQ(qos.durability(), eprosima::fastrtps::rtps::DurabilityKind_t::TRANSIENT);
    }

    // Persistent
    {
        DurabilityKind durability = eprosima::fastrtps::rtps::DurabilityKind_t::PERSISTENT;
        ReliabilityKind reliability = default_reliability_kind();

        QoS qos(durability, reliability);

        ASSERT_EQ(qos.durability(), eprosima::fastrtps::rtps::DurabilityKind_t::PERSISTENT);
    }
}

/**
 * Test \c QoS \c reliability getter method
 *
 * CASES:
 *  Best effort
 *  Reliable
 */
TEST(QoSTest, reliability_getter)
{
    // Best effort
    {
        DurabilityKind durability = default_durability_kind();
        ReliabilityKind reliability = eprosima::fastrtps::rtps::ReliabilityKind_t::BEST_EFFORT;

        QoS qos(durability, reliability);

        ASSERT_EQ(qos.reliability(), eprosima::fastrtps::rtps::ReliabilityKind_t::BEST_EFFORT);
    }

    // Reliable
    {
        DurabilityKind durability = default_durability_kind();
        ReliabilityKind reliability = eprosima::fastrtps::rtps::ReliabilityKind_t::RELIABLE;

        QoS qos(durability, reliability);

        ASSERT_EQ(qos.reliability(), eprosima::fastrtps::rtps::ReliabilityKind_t::RELIABLE);
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
