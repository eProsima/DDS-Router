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

#include <cpp_utils/testing/gtest_aux.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <fastdds/rtps/common/CacheChange.h>
#include <fastdds/rtps/common/SerializedPayload.h>

#include <ddspipe_core/efficiency/payload/PayloadPool.hpp>
#include <cpp_utils/exception/InconsistencyException.hpp>
#include <ddspipe_core/interface/IRoutingData.hpp>

using namespace eprosima::ddspipe::core;
using namespace eprosima::ddspipe::core::types;

// Using for gmock
using ::testing::_;
using ::testing::Invoke;
using ::testing::Return;

namespace eprosima {
namespace fastrtps {
namespace rtps {

/*
 * WORKAROUND:
 * This definition is needed due to googletest-distribution (1.11.0) requires to every class used inside ASSERT macro
 * to have the operator << defined in SAME namespace than the class.
 * In our case, Payload is defined as eprosima::fastrtps::rtps::SerializedPayload_t but redefined as
 * eprosima::ddspipe::core::types::Payload and the operator << is defined in eprosima::ddspipe::core::types
 * Thus, gtest could not find this definition (arising a very messy and cryptic compilation error).
 * This definition corrects that problem.
 *
 * NOTE:
 * In googletest-distribution release-1.10.0 this does not happen.
 */
void PrintTo(
        const SerializedPayload_t,
        std::ostream* os)
{
    *os << "::eprosima::fastrtps::rtps::SerializedPayload_t";
}

} /* namespace rtps */
} /* namespace fastrtps */
} /* namespace eprosima */

namespace eprosima {
namespace ddspipe {
namespace core {
namespace test {

/**
 * @brief Mock over Payload Pool re-implementing the needed methods returning error and
 * implementing public access to private variables.
 *
 */
class MockPayloadPool : public PayloadPool
{
public:

    // Using parent methods and variables as public
    using PayloadPool::PayloadPool;
    using PayloadPool::get_payload;
    using PayloadPool::release_payload;
    using PayloadPool::reserve_;
    using PayloadPool::release_;
    using PayloadPool::reserve_count_;
    using PayloadPool::release_count_;

    // Mock this virtual methods not implemented in parent class
    MOCK_METHOD(
        bool,
        get_payload,
        (uint32_t size, eprosima::ddspipe::core::types::Payload& target_payload),
        (override));

    MOCK_METHOD(
        bool,
        get_payload,
        (
            const Payload& src_payload,
            eprosima::fastrtps::rtps::IPayloadPool*&data_owner,
            eprosima::ddspipe::core::types::Payload& target_payload
        ),
        (override));

    MOCK_METHOD(
        bool,
        release_payload,
        (eprosima::ddspipe::core::types::Payload& target_payload),
        (override));
};

} /* namespace test */
} /* namespace core */
} /* namespace ddspipe */
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

        ASSERT_EQ(payload.max_size, 0u);
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

        ASSERT_EQ(payload.max_size, 0u);
        ASSERT_EQ(payload.data, nullptr);

        ASSERT_TRUE(pool.reserve_(sizeof(PayloadUnit) * 0x1000, payload));

        ASSERT_EQ(payload.max_size, sizeof(PayloadUnit) * 0x1000);
        ASSERT_NE(payload.data, nullptr);

        // This would (maybe) fail with SEG FAULT if the data has not been correctly set
        payload.data[0] = 4u;
        payload.data[0x1000 - 1] = 5u;
    }

    // 0 size
    {
        test::MockPayloadPool pool;
        Payload payload;

        ASSERT_EQ(payload.max_size, 0u);

        ASSERT_FALSE(pool.reserve_(0, payload));

        ASSERT_EQ(payload.max_size, 0u);
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

        ASSERT_EQ(payload.max_size, 0u);
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

        ASSERT_EQ(payload.max_size, 0u);
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
    for (unsigned int i = 0; i < 5u; ++i)
    {
        ASSERT_EQ(pool.reserve_count_, i);
        pool.reserve_(sizeof(PayloadUnit), payloads[i]);
    }
    ASSERT_EQ(pool.reserve_count_, 5u);

    // release 4 values
    for (unsigned int i = 0; i < 4u; ++i)
    {
        ASSERT_EQ(pool.release_count_, i);
        pool.release_(payloads[i]);
    }
    ASSERT_EQ(pool.release_count_, 4u);

    // store 5 values
    for (unsigned int i = 5; i < 10u; ++i)
    {
        ASSERT_EQ(pool.reserve_count_, i);
        pool.reserve_(sizeof(PayloadUnit), payloads[i]);
    }
    ASSERT_EQ(pool.reserve_count_, 10u);

    // release 6 values
    for (unsigned int i = 4; i < 10u; ++i)
    {
        ASSERT_EQ(pool.release_count_, i);
        pool.release_(payloads[i]);
    }
    ASSERT_EQ(pool.release_count_, 10u);

    // release more values than reserved
    ASSERT_THROW(pool.release_(payloads[10]), eprosima::utils::InconsistencyException);
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

        // Clean cache change correctly so process dont break
        cc.payload_owner(nullptr);
    }

    // get_payload for payload goes ok
    {
        test::MockPayloadPool pool;
        eprosima::fastrtps::rtps::CacheChange_t cc;

        EXPECT_CALL(pool, get_payload(_, _)).Times(1).WillOnce(Return(false));

        EXPECT_FALSE(pool.get_payload(sizeof(PayloadUnit), cc));
    }
}

/**
 * Test get_payload cache_change with source fails if the child class fails
 *
 * CASES:
 * - get_payload for payload goes ok
 * - get_payload for payload fails
 */
TEST(PayloadPoolTest, get_payload_from_src_cache_change)
{
    // get_payload for payload goes ok
    {
        test::MockPayloadPool pool;
        eprosima::fastrtps::rtps::CacheChange_t target;
        Payload source;
        eprosima::fastrtps::rtps::IPayloadPool* aux_pool;

        EXPECT_CALL(pool, get_payload(_, _, _)).Times(1).WillOnce(Return(true));

        EXPECT_TRUE(pool.get_payload(source, aux_pool, target));

        // Clean cache change correctly so process dont break
        target.payload_owner(nullptr);
    }

    // get_payload for payload goes ok
    {
        test::MockPayloadPool pool;
        eprosima::fastrtps::rtps::CacheChange_t target;
        Payload source;
        eprosima::fastrtps::rtps::IPayloadPool* aux_pool;

        EXPECT_CALL(pool, get_payload(_, _, _)).Times(1).WillOnce(Return(false));

        EXPECT_FALSE(pool.get_payload(source, aux_pool, target));
    }
}

/**
 * Test release_payload cache_change method using MockPayloadPool when inside get_payload method fails
 *
 * CASES:
 *  different ownership
 *  this ownership release ok
 *  this ownership release fail
 */
TEST(PayloadPoolTest, release_payload_cache_change)
{
    // different ownership
    {
        test::MockPayloadPool pool;
        eprosima::fastrtps::rtps::CacheChange_t cc;
        cc.payload_owner(nullptr);

        EXPECT_THROW(pool.release_payload(cc), eprosima::utils::InconsistencyException);
    }

    // this ownership release ok
    {
        test::MockPayloadPool pool;
        eprosima::fastrtps::rtps::CacheChange_t cc;

        // ownership must be this pool
        cc.payload_owner(&pool);

        EXPECT_CALL(pool, release_payload(_)).Times(1).WillOnce(Return(true));

        EXPECT_TRUE(pool.release_payload(cc));
    }

    // this ownership release fail
    {
        test::MockPayloadPool pool;
        eprosima::fastrtps::rtps::CacheChange_t cc;

        // ownership must be this pool
        cc.payload_owner(&pool);

        EXPECT_CALL(pool, release_payload(_)).Times(1).WillOnce(Return(false));

        EXPECT_FALSE(pool.release_payload(cc));

        // Clean cache change correctly so process dont break
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
