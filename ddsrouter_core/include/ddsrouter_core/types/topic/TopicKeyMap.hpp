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
 * @file TopicKeyMap.hpp
 */

#ifndef _DDSROUTERCORE_TYPES_TOPICKEYMAP_HPP_
#define _DDSROUTERCORE_TYPES_TOPICKEYMAP_HPP_

#include <ddsrouter_utils/exception/InconsistencyException.hpp>
#include <ddsrouter_core/types/topic/Topic.hpp>

#include <ddsrouter_utils/exception/InitializationException.hpp>

#include <memory>
#include <list>
#include <vector>
#include <mutex> // NOTE performance: Replace by <shared_mutex> if C++17 available

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

// NOTE performance: Temorary alias due to current C++14 limitation. Should be a std::shared_mutex if C++17
using ShMutexT = std::mutex; // std::shared_mutex

// NOTE performance: Temorary alias due to current C++14 limitation. Should be a std::shared_lock if C++17
template <typename MT>
using ShLockT = std::lock_guard<MT>; // std::shared_lock

//! Default number of buckets (prime number preferred)
constexpr unsigned int DEFAULT_NUM_BUCKETS = 199;

//! Max number of buckets
constexpr unsigned int MAX_NUM_BUCKETS = 10000;

/**
 * Compile-time predicate validating std::unique_ptr or std::shared_ptr types only.
 */
template <typename T>
struct IsSharedOrUniquePointer : std::false_type {};

template <typename T, typename D>
struct IsSharedOrUniquePointer<std::unique_ptr<T, D>> : std::true_type {};

template <typename T>
struct IsSharedOrUniquePointer<std::shared_ptr<T>> : std::true_type {};

/**
 * @brief Simple generic thread-safe hash table to map owned elements from a RealTopic key type
 *
 */
template <typename OwnMappedT>
class TopicKeyMap
{
private:

    //! Only shared or unique pointers allowed
    static_assert( IsSharedOrUniquePointer<OwnMappedT>::value );

    using NotOwnMappedT = typename OwnMappedT::pointer;

    //! TODO performance: Provisional hash function just adding up topic fields hashes
    struct HashT
    {
        std::hash<std::string> hash_str_;
        std::hash<bool> hash_bool_;
        std::size_t operator ()(
                const RealTopic& topic) const
        {
            return hash_str_(topic.name()) +
                   hash_str_(topic.type()) +
                   hash_bool_(topic.has_key()) +
                   hash_bool_(topic.is_reliable());
        }

    };

public:

    /**
     * @brief Initializes the container
     *
     * @param num_buckets Number of buckets. Should be a prime number to reduce bucket collisions
     * @param in_hasher Hasher functor
     *
     */
    TopicKeyMap(
            unsigned int num_buckets = DEFAULT_NUM_BUCKETS,
            const HashT& in_hasher = HashT())
        : buckets_(num_buckets)
        , hasher_(in_hasher)
    {
        if (num_buckets == 0)
        {
            throw utils::InitializationException("Cannot set 0 number of buckets in TopicKeyMap");
        }
        if (num_buckets > MAX_NUM_BUCKETS)
        {
            throw utils::InitializationException("Maximum number of buckets exceeded TopicKeyMap");
        }
        for (auto i = 0u; i < num_buckets; ++i)
        {
            buckets_[i] = std::make_unique<Bucket>();
        }
    }

    TopicKeyMap(
            const TopicKeyMap& other) = delete;
    TopicKeyMap& operator =(
            const TopicKeyMap& other) = delete;

    /**
     * @brief Topic mapped value getter
     *
     * @param topic Real topic
     *
     * @return Mapped value
     */
    NotOwnMappedT get(
            const RealTopic& topic)
    {

        return get_bucket_(topic).value_for(topic);
    }

    /**
     * @brief Topic value existence
     *
     * @param topic Real topic
     *
     * @return Whether the topic has a mapped value
     */
    bool exists(
            const RealTopic& topic)
    {
        return get_bucket_(topic).value_for(topic) != nullptr;
    }

    /**
     * @brief Topic insertion
     *
     * @param topic Real topic
     * @param new_mapped Value to insert as a owned pointer
     *
     * @return Raw pointer of the inserted value
     *
     * @throw If topic has already been inserted
     */
    NotOwnMappedT insert(
            const RealTopic& topic,
            OwnMappedT&& new_mapped)
    {
        auto new_mapped_raw = new_mapped.get();

        this->get_bucket_(topic).add_mapped(topic, std::move(new_mapped));

        return new_mapped_raw;
    }

    /**
     * @brief Topic value pop
     *
     * @param topic Real topic
     *
     * @return Owned pointer of the inserted value associated to the topic
     */
    OwnMappedT pop(
            const RealTopic& topic)
    {
        return this->get_bucket_(topic).pop_mapped(topic);
    }

private:

    /**
     * @brief Bucket holding owned objects in a hash table
     *
     */
    class Bucket
    {
    private:

        using BucketValueT = std::pair<RealTopic, OwnMappedT>;
        using BucketDataT = std::list<BucketValueT>;
        using BucketIteratorT = typename BucketDataT::iterator;

        BucketDataT data_;
        mutable ShMutexT mutex_;

        BucketIteratorT find_entry_for_(
                const RealTopic& topic)
        {
            return std::find_if(std::begin(this->data_), std::end(this->data_),
                           [&](const BucketValueT& item)
                           {
                               return std::get<RealTopic>(item) == topic;
                           });
        }

    public:

        NotOwnMappedT value_for(
                const RealTopic& topic)
        {

            ShLockT<ShMutexT> lock(mutex_);

            const BucketIteratorT found_entry = find_entry_for_(topic);

            return (found_entry == data_.end() ? nullptr : std::get<OwnMappedT>(*found_entry).get());
        }

        void add_mapped(
                const RealTopic& topic,
                OwnMappedT&& new_mapped)
        {
            std::unique_lock<ShMutexT> lock(this->mutex_);

            const BucketIteratorT found_entry = this->find_entry_for_(topic);

            if (found_entry == this->data_.end())
            {
                this->data_.emplace_back( std::make_pair(topic, std::move(new_mapped)));

            }
            else
            {
                throw utils::InconsistencyException(
                          utils::Formatter() << " mapped object with topic " << topic <<
                              " already existing");
            }
        }

        OwnMappedT pop_mapped(
                const RealTopic& topic)
        {

            std::unique_lock<ShMutexT> lock(this->mutex_);

            const BucketIteratorT found_entry = this->find_entry_for_(topic);

            if (found_entry != this->data_.end())
            {

                auto popped_mapped = std::move(std::get<OwnMappedT>(*found_entry));

                this->data_.erase(found_entry);

                return popped_mapped;

            }
            else
            {
                // No mapped object with this topic
                return nullptr;
            }
        }

    };

private:

    ////////
    // Auxiliary methods

    //! Get a bucket of this topic
    Bucket& get_bucket_(
            const RealTopic& topic)
    {
        return *(buckets_[hasher_(topic) % buckets_.size()]);
    }

    ////////
    // Members

    //! Vector of Bucket instances
    std::vector< std::unique_ptr<Bucket>> buckets_;

    //! Hash functor to compute hashed keys
    HashT hasher_;
};

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_TOPICKEYMAP_HPP_*/
