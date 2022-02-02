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

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <gtest_aux.hpp>

#include <fastdds/rtps/common/CacheChange.h>

#include <ddsrouter/communication/payload_pool/PayloadPool.hpp>
#include <ddsrouter/exceptions/InconsistencyException.hpp>
#include <ddsrouter/types/Data.hpp>

using namespace eprosima::ddsrouter;

// Using for gmock
using ::testing::_;
using ::testing::Invoke;
using ::testing::Return;
using ::testing::Throw;
using ::testing::AnyNumber;

std::ostream& operator <<(
        std::ostream& os,
        const eprosima::ddsrouter::Payload& payload)
{
    eprosima::ddsrouter::operator<<(os, payload);
    return os;
}

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

    using PayloadPool::get_payload;
    using PayloadPool::release_payload;
    using PayloadPool::reserve_;
    using PayloadPool::release_;
    using PayloadPool::reserve_count_;
    using PayloadPool::release_count_;


    // MOCK_METHOD2(get_payload, bool(
    //             uint32_t size,
    //             Payload& payload));

    // MOCK_METHOD3(get_payload, bool(
    //             const Payload& src_payload,
    //             IPayloadPool*& data_owner,
    //             Payload& target_payload));

    // MOCK_METHOD1(release_payload, bool(
    //             Payload& target_payload));

    MOCK_METHOD(
        bool,
        get_payload,
        (uint32_t size, eprosima::ddsrouter::Payload& target_payload),
        (override));

    MOCK_METHOD(
        bool,
        get_payload,
        (const Payload& src_payload, IPayloadPool*& data_owner, eprosima::ddsrouter::Payload& target_payload),
        (override));

    MOCK_METHOD(
        bool,
        release_payload,
        (eprosima::ddsrouter::Payload& target_payload),
        (override));

    // bool get_payload(
    //         uint32_t size,
    //         Payload& payload){ return false;}

    // bool get_payload(
    //         const Payload& src_payload,
    //         IPayloadPool*& data_owner,
    //         Payload& target_payload){return false;}
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
        ASSERT_EQ(pool.reserve_count_, i);
        pool.reserve_(sizeof(PayloadUnit), payloads[i]);
    }
    ASSERT_EQ(pool.reserve_count_, 5);

    // release 4 values
    for (int i=0; i<4; ++i)
    {
        ASSERT_EQ(pool.release_count_, i);
        pool.release_(payloads[i]);
    }
    ASSERT_EQ(pool.release_count_, 4);

    // store 5 values
    for (int i=5; i<10; ++i)
    {
        ASSERT_EQ(pool.reserve_count_, i);
        pool.reserve_(sizeof(PayloadUnit), payloads[i]);
    }
    ASSERT_EQ(pool.reserve_count_, 10);

    // release 6 values
    for (int i=4; i<10; ++i)
    {
        ASSERT_EQ(pool.release_count_, i);
        pool.release_(payloads[i]);
    }
    ASSERT_EQ(pool.release_count_, 10);

    // release more values than reserved
    ASSERT_THROW(pool.release_(payloads[10]), InconsistencyException);
}

/**
 * Test clean method
 *
 * STEPS:
 *  start clean
 *  reserve and not clean
 *  release and clean
 */
TEST(PayloadPoolTest, is_clean)
{
    test::MockPayloadPool pool;

    // start clean
    ASSERT_TRUE(pool.is_clean());

    // reserve and not clean
    eprosima::fastrtps::rtps::CacheChange_t cc;
    pool.reserve_(sizeof(PayloadUnit), cc.serializedPayload);
    ASSERT_FALSE(pool.is_clean());

    // release and clean
    pool.release_(cc.serializedPayload);
    ASSERT_TRUE(pool.is_clean());
}

/**
 * Test get_payload cache_change fails if the child class fails
 *
 * CASES:
 * - get_payload for payload goes ok
 * - get_payload for payload fails
 */
TEST(PayloadPoolTest, get_payload_cache_change)
{
    // get_payload for payload goes ok
    {
        test::MockPayloadPool pool;
        eprosima::fastrtps::rtps::CacheChange_t cc;

        EXPECT_CALL(pool, get_payload(_, _)).Times(1).WillOnce(Return(true));

        EXPECT_TRUE(pool.get_payload(sizeof(PayloadUnit), cc));

        // Destroy payload correctly or process will be killed
        pool.release_payload(cc);
        cc.payload_owner(nullptr);
    }
}

// /**
//  * Test get_payload copy cache_change fails if the child class fails
//  */
// TEST(PayloadPoolTest, get_payload_from_src_cache_change)
// {
//     eprosima::fastrtps::rtps::IPayloadPool* pool = new test::MockPayloadPool(); // Requires to be ptr to pass it to get_payload
//     eprosima::fastrtps::rtps::CacheChange_t source;
//     eprosima::fastrtps::rtps::CacheChange_t target;

//     ASSERT_FALSE(
//         pool->get_payload(
//             source.serializedPayload,
//             pool,
//             target));
// }

/**
 * Test release_payload cache_change method using MockPayloadPool when inside get_payload method fails
 *
 * CASES:
 *  different ownership
 *  this ownership
 */
TEST(PayloadPoolTest, release_payload_cache_change)
{
    // // different ownership
    // {
    //     test::MockPayloadPool pool;
    //     eprosima::fastrtps::rtps::CacheChange_t cc;

    //     cc.payload_owner(nullptr);

    //     ASSERT_THROW(pool.release_payload(cc), InconsistencyException);
    // }

    // // this ownership
    // {
    //     test::MockPayloadPool pool;
    //     eprosima::fastrtps::rtps::CacheChange_t cc;

    //     cc.payload_owner(&pool);

    //     ASSERT_FALSE(pool.release_payload(cc));

    //     // As the method fails, the ownership must be set to nullptr before destroying the change
    //     cc.payload_owner(nullptr);
    // }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
