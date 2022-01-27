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

#include <fastdds/rtps/common/CacheChange.h>

#include <ddsrouter/communication/payload_pool/PayloadPool.hpp>
#include <ddsrouter/exceptions/InconsistencyException.hpp>

using namespace eprosima::ddsrouter;

namespace eprosima {
namespace ddsrouter {
namespace test {

/**
 * @brief Mock over Payload Pool re-implementing the needed methods returning error and
 * implementing public access to private variables.
 *
 */
class MockPayloadPool : public PayloadPool
{
public:
    using PayloadPool::PayloadPool;

    bool get_payload(
            uint32_t size,
            Payload& payload) override
    {
        return false;
    }

    bool get_payload(
            const Payload& src_payload,
            IPayloadPool*& data_owner,
            Payload& target_payload) override
    {
        return false;
    }

    // These are required in order to use father methods overloaded here
    using PayloadPool::release_payload;
    using PayloadPool::get_payload;

    bool release_payload(
            Payload& payload) override
    {
        return false;
    }

    bool reserve_(
            uint32_t size,
            Payload& payload) override
    {
        return PayloadPool::reserve_(size, payload);
    }

    bool release_(
            Payload& payload) override
    {
        return PayloadPool::release_(payload);
    }

    uint64_t reserve_count()
    {
        return reserve_count_.load();
    }

    uint64_t release_count()
    {
        return release_count_.load();
    }
};

} /* namespace test */
} /* namespace ddsrouter */
} /* namespace eprosima */

/**
 * Test reserve_ method
 *
 * CASES:
 *  small size
 *  large size
 *  0 size
 */
TEST(PayloadPoolTest, reserve)
{
    // small size
    {
        test::MockPayloadPool pool;
        Payload payload;

        ASSERT_EQ(payload.max_size, 0);
        ASSERT_EQ(payload.data, nullptr);

        ASSERT_TRUE(pool.reserve_(sizeof(PayloadUnit), payload));

        ASSERT_EQ(payload.max_size, sizeof(PayloadUnit));
        ASSERT_NE(payload.data, nullptr);

        // This would (maybe) fail with SEG FAULT if the data has not been correctly set
        payload.data[0] = 16u;
    }

    // large size
    {
        test::MockPayloadPool pool;
        Payload payload;

        ASSERT_EQ(payload.max_size, 0);
        ASSERT_EQ(payload.data, nullptr);

        ASSERT_TRUE(pool.reserve_(sizeof(PayloadUnit) * 0x1000, payload));

        ASSERT_EQ(payload.max_size, sizeof(PayloadUnit) * 0x1000);
        ASSERT_NE(payload.data, nullptr);

        // This would (maybe) fail with SEG FAULT if the data has not been correctly set
        payload.data[0] = 4u;
        payload.data[0x1000] = 5u;
    }

    // 0 size
    {
        test::MockPayloadPool pool;
        Payload payload;

        ASSERT_EQ(payload.max_size, 0);

        ASSERT_FALSE(pool.reserve_(0, payload));

        ASSERT_EQ(payload.max_size, 0);
    }
}

/**
 * Test release_ method
 *
 * CASES:
 *  small size
 *  large size
 */
TEST(PayloadPoolTest, release)
{
    // small size
    {
        test::MockPayloadPool pool;
        Payload payload;
        pool.reserve_(sizeof(PayloadUnit), payload);

        ASSERT_EQ(payload.max_size, sizeof(PayloadUnit));
        ASSERT_NE(payload.data, nullptr);

        ASSERT_TRUE(pool.release_(payload));

        ASSERT_EQ(payload.max_size, 0);
        ASSERT_EQ(payload.data, nullptr);
    }

    // large size
    {
        test::MockPayloadPool pool;
        Payload payload;
        pool.reserve_(sizeof(PayloadUnit) * 0x1000, payload);

        ASSERT_EQ(payload.max_size, sizeof(PayloadUnit) * 0x1000);
        ASSERT_NE(payload.data, nullptr);

        ASSERT_TRUE(pool.release_(payload));

        ASSERT_EQ(payload.max_size, 0);
        ASSERT_EQ(payload.data, nullptr);
    }
}

/**
 * Test release_ method
 *
 * STEPS:
 *  store 5 values
 *  release 4 values
 *  store 5 more values
 *  release 6 values
 *  release more values than reserved
 */
TEST(PayloadPoolTest, reserve_and_release_counter)
{
    test::MockPayloadPool pool;
    std::vector<Payload> payloads(11);

    // store 5 values
    for (int i=0; i<5; ++i)
    {
        ASSERT_EQ(pool.reserve_count(), i);
        pool.reserve_(sizeof(PayloadUnit), payloads[i]);
    }
    ASSERT_EQ(pool.reserve_count(), 5);

    // release 4 values
    for (int i=0; i<4; ++i)
    {
        ASSERT_EQ(pool.release_count(), i);
        pool.release_(payloads[i]);
    }
    ASSERT_EQ(pool.release_count(), 4);

    // store 5 values
    for (int i=5; i<10; ++i)
    {
        ASSERT_EQ(pool.reserve_count(), i);
        pool.reserve_(sizeof(PayloadUnit), payloads[i]);
    }
    ASSERT_EQ(pool.reserve_count(), 10);

    // release 6 values
    for (int i=4; i<10; ++i)
    {
        ASSERT_EQ(pool.release_count(), i);
        pool.release_(payloads[i]);
    }
    ASSERT_EQ(pool.release_count(), 10);

    // release more values than reserved
    ASSERT_THROW(pool.release_(payloads[10]), InconsistencyException);
}

/**
 * Test get_payload cache_change fails if the child class fails
 */
TEST(PayloadPoolTest, get_payload_negative)
{
    test::MockPayloadPool pool;
    eprosima::fastrtps::rtps::CacheChange_t cc;

    ASSERT_FALSE(pool.get_payload(sizeof(PayloadUnit), cc));
}

/**
 * Test get_payload copy cache_change fails if the child class fails
 */
TEST(PayloadPoolTest, get_payload_from_src_negative)
{
    eprosima::fastrtps::rtps::IPayloadPool* pool = new test::MockPayloadPool(); // Requires to be ptr to pass it to get_payload
    eprosima::fastrtps::rtps::CacheChange_t source;
    eprosima::fastrtps::rtps::CacheChange_t target;

    ASSERT_FALSE(
        pool->get_payload(
            source.serializedPayload,
            pool,
            target));
}

/**
 * Test release_payload cache_change method using MockPayloadPool when inside get_payload method fails
 *
 * CASES:
 *  different ownership
 *  this ownership
 */
TEST(PayloadPoolTest, release_payload_negative)
{
    // different ownership
    {
        test::MockPayloadPool pool;
        eprosima::fastrtps::rtps::CacheChange_t cc;

        cc.payload_owner(nullptr);

        ASSERT_THROW(pool.release_payload(cc), InconsistencyException);
    }

    // this ownership
    {
        test::MockPayloadPool pool;
        eprosima::fastrtps::rtps::CacheChange_t cc;

        cc.payload_owner(&pool);

        ASSERT_FALSE(pool.release_payload(cc));

        // As the method fails, the ownership must be set to nullptr before destroying the change
        cc.payload_owner(nullptr);
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
