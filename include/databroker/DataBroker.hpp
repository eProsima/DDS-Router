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
 * @file DataBroker.hpp
 *
 */

#ifndef EPROSIMA_DATABROKER_DATABROKER_HPP
#define EPROSIMA_DATABROKER_DATABROKER_HPP

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <vector>

#include <FileWatch.hpp>

#include <databroker/DataBrokerListener.hpp>
#include <databroker/DataBrokerConfiguration.hpp>
#include <databroker/DataBrokerParticipant.hpp>
#include <databroker/DataBrokerLocalParticipant.hpp>
#include <databroker/DataBrokerWANParticipant.hpp>
#include <databroker/Address.hpp>

namespace eprosima {
namespace databroker {

enum Command
{
    UNKNOWN,
    ERROR,
    CMD_HELP,
    ADD_TOPIC,
    REMOVE_TOPIC,
    LOAD_FILE,
    RELOAD_FILE,
    EXIT,
    VOID
};

static const std::map<Command, std::list<std::string>> COMMAND_KEYWORDS =
{
    {Command::CMD_HELP, {"h", "help"}},
    {Command::ADD_TOPIC, {"a", "add"}},
    {Command::REMOVE_TOPIC, {"r", "rm", "remove"}},
    {Command::LOAD_FILE, {"l", "load"}},
    {Command::RELOAD_FILE, {"rl", "reload"}},
    {Command::EXIT, {"e", "q", "exit", "quit"}}
};

static const std::map<Command, std::vector<std::string>> COMMAND_ARGUMENTS =
{
    {Command::CMD_HELP, {}},
    {Command::RELOAD_FILE, {}},
    {Command::EXIT, {}},
    {Command::ADD_TOPIC, {"topic_name"}},
    {Command::REMOVE_TOPIC, {"topic_name"}},
    {Command::LOAD_FILE, {"file_path"}},
};

class DataBroker
{
public:

    DataBroker(
            DataBrokerConfiguration configuration);

    virtual ~DataBroker();

    bool init();

    // 0 seconds means forever
    bool run();

    static void stop();

protected:

    void add_topic_(
            const std::string& topic);

    void remove_topic_(
            const std::string& topic);

    bool run_interactive();
    bool run_time(
            const uint32_t seconds);

    Command read_command(
            const std::string& input,
            std::vector<std::string>& args);

    void print_help();

    void stop_all_topics();

    static bool is_stopped();

    bool start_watch_file_();

    bool finish_watch_file_();

    bool reload_configuration_file_(
            const std::string& path = "");

private:

    DataBrokerLocalParticipant* local_;
    DataBrokerWANParticipant* wan_;

    DataBrokerListener listener_;

    DataBrokerConfiguration configuration_;

    std::map<std::string, bool> topics_;

    bool enabled_;

    static std::atomic<bool> stop_;
    static std::mutex stop_mutex_cv_;
    static std::condition_variable stop_cv_;

    filewatch::FileWatch<std::string>* file_watch_handler_;

    std::recursive_mutex topics_mutex_;
};

} /* namespace databroker */
} /* namespace eprosima */

#endif /* EPROSIMA_DATABROKER_DATABROKER_HPP */
