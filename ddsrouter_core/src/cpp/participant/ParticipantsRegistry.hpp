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
 * @file ParticipantsRegistry.hpp
 */

#ifndef __SRC_DDSROUTERCORE_CORE_PARTICIPANTSREGISTRY_HPP_
#define __SRC_DDSROUTERCORE_CORE_PARTICIPANTSREGISTRY_HPP_

#include <list>
#include <vector>
#include <algorithm>
#include <mutex> // NOTE performance: Replace by <shared_mutex> if C++17 available

#include <ddsrouter_utils/exception/ConfigurationException.hpp>
#include <ddsrouter_utils/exception/UnsupportedException.hpp>
#include <ddsrouter_utils/Log.hpp>
#include <ddsrouter_utils/utils.hpp>

#include <ddsrouter_core/configuration/participant/ParticipantConfiguration.hpp>

#include <participant/auxiliar/DummyParticipant.hpp>
#include <participant/auxiliar/EchoParticipant.hpp>
#include <participant/auxiliar/BlankParticipant.hpp>
#include <participant/rtps/SimpleParticipant.hpp>
#include <participant/rtps/LocalDiscoveryServerParticipant.hpp>
#include <participant/rtps/WANParticipant.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

constexpr unsigned int DEFAULT_NUM_BUCKETS = 199; // Prime number

/**
 * Functor with a call operator receiving an object containing a ParticipantId
 */
template <typename IdHolderT>
struct ExtractorId
{
    const types::ParticipantId& operator ()(
            const IdHolderT& id_holder);
};

//! Specialization when IdHolderT is just a ParticipantId
template <>
struct ExtractorId< types::ParticipantId >
{
    const types::ParticipantId& operator ()(
            const types::ParticipantId& id_holder)
    {
        return id_holder;
    }

};

//! Specialization when IdHolderT is a ParticipantConfiguration
template <>
struct ExtractorId< configuration::ParticipantConfiguration >
{
    const types::ParticipantId& operator ()(
            const configuration::ParticipantConfiguration& cfg)
    {
        return cfg.id();
    }

};

//! Specialization when IdHolderT is a owned pointer of a type T exposing a non-static method `ParticipantId T::id()`
template <typename T>
struct ExtractorId< std::unique_ptr<T>>
{
    const types::ParticipantId& operator ()(
            const std::unique_ptr<T>& id_holder)
    {
        return id_holder->id();
    }

};

//! Template helper to clean const and reference qualifiers. NOTE: similar to C++20's std::remove_cvref_t
template <typename T>
using remove_const_ref_t = std::remove_const_t<std::remove_reference_t<T>>;

// NOTE performance: Temorary alias due to current C++14 limitation. Should be a std::shared_mutex if C++17
using ShMutexT = std::mutex; // std::shared_mutex

// NOTE performance: Temorary alias due to current C++14 limitation. Should be a std::shared_lock if C++17
template <typename MT>
using ShLockT = std::lock_guard<MT>; // std::shared_lock

/**
 * @brief Simple thread-safe hash table to store collectively all the Participants of the Router.
 *
 */
class ParticipantsRegistry
{
private:

    //! ParticipantT is a pointer to participant, to be returned to users
    using ParticipantT = std::add_pointer_t<IParticipant>;

    //! OwnParticipantT is a owned pointer to Participant, to be privately managed within this Registry
    using OwnParticipantT = std::unique_ptr<IParticipant>;

    //! Default hash functor to classify Participants into buckets by their ParticipantName
    using HashT = std::hash<types::ParticipantName>;

public:

    /**
     * @brief Single constructor
     *
     * @param num_buckets Number of buckets. Should be a prime number to reduce bucket collisions
     * @param in_hasher Hasher functor
     *
     */
    ParticipantsRegistry(
            unsigned int num_buckets = DEFAULT_NUM_BUCKETS,
            const HashT& in_hasher = HashT())
        : buckets_(num_buckets)
        , hasher_(in_hasher)
    {
        for (auto i = 0u; i < num_buckets; ++i)
        {
            buckets_[i] = std::make_unique<ParticipantsBucket>();
        }
    }

    ParticipantsRegistry(
            const ParticipantsRegistry& other) = delete;
    ParticipantsRegistry& operator =(
            const ParticipantsRegistry& other) = delete;

    /**
     * @brief Returns a participant
     *
     * @return Non-owning participant
     */
    ParticipantT get_participant(
            const types::ParticipantName& name)
    {
        return this->get_participants_bucket_(name)->value_for(name);
    }

    /**
     * @brief Creates and adds a participant from arguments
     *
     * @param id_holder Holder of a ParticipantId, to be extracted by ExtractorId
     * @param args Arguments to forward to the constructor
     *
     * @throw InitializationException if kind is invalid
     * @throw InitializationException if kind is invalid
     *
     * @return Non-owning participant just created
     */
    template <typename IdHolderT, typename ... ConstructorArgs>
    ParticipantT add_participant(
            IdHolderT&& id_holder,
            ConstructorArgs&&... args)
    {

        const types::ParticipantId id = ExtractorId<remove_const_ref_t<IdHolderT>>()(id_holder);

        logInfo(DDSROUTER, "Making new participant" << id);

        std::unique_ptr<IParticipant> new_participant;

        switch (id.kind())
        {
            case types::ParticipantKind::blank:
            {
                // BlankParticipant
                new_participant = std::make_unique<BlankParticipant>(std::forward<IdHolderT>(id_holder),
                                std::forward<ConstructorArgs>(args) ... );
                break;
            }
            case types::ParticipantKind::echo:
            {
                // EchoParticipant
                new_participant = std::make_unique<EchoParticipant>(std::forward<IdHolderT>(id_holder),
                                std::forward<ConstructorArgs>(args) ... );
                break;
            }
            case types::ParticipantKind::dummy:
            {
                // DummyParticipant
                new_participant = std::make_unique<DummyParticipant>(std::forward<IdHolderT>(id_holder),
                                std::forward<ConstructorArgs>(args) ... );
                break;
            }
            case types::ParticipantKind::simple_rtps:
                // Simple RTPS Participant
            {
                new_participant = std::make_unique<rtps::SimpleParticipant>(std::forward<IdHolderT>(
                                    id_holder), std::forward<ConstructorArgs>(args) ... );
                break;
            }
            case types::ParticipantKind::local_discovery_server:
                // Discovery Server RTPS Participant
            {
                new_participant =
                        std::make_unique<rtps::LocalDiscoveryServerParticipant>(std::forward<IdHolderT>(
                                    id_holder), std::forward<ConstructorArgs>(args) ... );
                break;
            }
            case types::ParticipantKind::wan:
                // Discovery Server RTPS Participant
            {
                new_participant = std::make_unique<rtps::WANParticipant>(std::forward<IdHolderT>(
                                    id_holder), std::forward<ConstructorArgs>(args) ... );
                break;
            }
            default:
            {
                // This should not happen as every kind must be in the switch

                throw utils::InitializationException(
                          utils::Formatter() << "Value of ParticipantKind out of enumeration." << id.kind());
            }
        }

        logInfo(DDSROUTER, "Made new participant" << *new_participant);

        auto new_participant_raw = new_participant.get();

        this->get_participants_bucket_(new_participant->id().name())->add_participant(std::move(new_participant));

        logInfo(DDSROUTER, "Participant added into registry " << new_participant_raw->id() << ".");

        return new_participant_raw;
    }

    /**
     * @brief Pop a participant
     *
     * Return the owned participant indexed by ParticipantName
     *
     * @param name Participant name
     *
     * @return Owned participant, may be null if not found
     */
    OwnParticipantT pop_participant(
            const types::ParticipantName& name)
    {
        return this->get_participants_bucket_(name)->pop_participant(name);
    }

private:

    /**
     * @brief Bucket holding participants in a hash table
     *
     */
    class ParticipantsBucket
    {
    private:

        using BucketValueT = std::pair<types::ParticipantName, OwnParticipantT>;
        using BucketDataT = std::list<BucketValueT>;
        using BucketIteratorT = typename BucketDataT::iterator;

        //! Holds all owned participants
        BucketDataT data_;

        //! Mutex for protecting accesses
        mutable ShMutexT mutex_;

        /**
         * @brief Returns iterator to owned participant
         *
         * @param name Participant name
         *
         * @return Iterator to the owner participant or std::end(data_)
         *
         */
        BucketIteratorT find_entry_for_(
                const types::ParticipantName& name)
        {
            return std::find_if(std::begin(this->data_), std::end(this->data_),
                           [&](const BucketValueT& item)
                           {
                               return std::get<types::ParticipantName>(item) == name;
                           });
        }

    public:

        /**
         * @brief Return
         *
         * @param name Participant name
         *
         * @return Owned participant, may be null if not found
         */
        ParticipantT value_for(
                const types::ParticipantName& name)
        {
            ShLockT<ShMutexT> lock(mutex_);
            const BucketIteratorT found_entry = find_entry_for_(name);
            return (found_entry == data_.end() ? nullptr : std::get<OwnParticipantT>(*found_entry).get());
        }

        /**
         * @brief Add a new participant if does not exist
         *
         * @param participant Owned participant
         *
         * @throw If participant with the same name already exists
         */
        void add_participant(
                OwnParticipantT&& participant)
        {
            const auto& name = participant->id().name();

            std::unique_lock<ShMutexT> lock(this->mutex_);

            const BucketIteratorT found_entry = this->find_entry_for_(name);

            if (found_entry == this->data_.end())
            {
                this->data_.emplace_back( std::make_pair(name, std::move(participant)));

            }
            else
            {
                throw utils::InconsistencyException(
                          utils::Formatter() << " participant with name " << name << " already existing");
            }
        }

        /**
         * @brief Pop a participant
         *
         * Return the owned participant indexed by ParticipantName
         *
         * @param name Participant name
         *
         * @return Owned participant, may be null if not found
         */
        OwnParticipantT pop_participant(
                const types::ParticipantName& name)
        {

            std::unique_lock<ShMutexT> lock(this->mutex_);

            const BucketIteratorT found_entry = this->find_entry_for_(name);

            if (found_entry != this->data_.end())
            {

                auto popped_participant = std::move(std::get<OwnParticipantT>(*found_entry));

                this->data_.erase(found_entry);

                return popped_participant;

            }
            else
            {
                // No participant with this name
                return nullptr;
            }
        }

    };

private:

    /**
     * @brief Return a bucket mapped by this participant name
     */
    ParticipantsBucket* get_participants_bucket_(
            const types::ParticipantName& name)
    {
        return buckets_[hasher_(name) % buckets_.size()].get();
    }

    /////
    // MEMBERS

    //! Vector of ParticipantBucket instances
    std::vector< std::unique_ptr<ParticipantsBucket>> buckets_;

    //! Hasher functor to map a Participant Name to a bucket
    HashT hasher_;
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_CORE_PARTICIPANTSREGISTRY_HPP_ */
