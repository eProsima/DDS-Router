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
 * @file DataBroker.cpp
 *
 */

#include <cctype>       // for using tolower
#include <stdlib.h>     // for using the function sleep

#include <fastdds/dds/log/Log.hpp>

#include <databroker/DataBroker.hpp>
#include <databroker/DataBrokerWANParticipant.hpp>
#include <databroker/DataBrokerConfiguration.hpp>
#include <databroker/utils.hpp>

namespace eprosima {
namespace databroker {

DataBroker::DataBroker(
        DataBrokerConfiguration configuration)
    : configuration_(configuration)
{
    logInfo(DATABROKER, "Creating DataBroker instance");

    wan_ = new DataBrokerWANParticipant(&listener_, configuration.wan_configuration);
    local_ = new DataBrokerLocalParticipant(&listener_, configuration.local_configuration);
}

DataBroker::~DataBroker()
{
    logInfo(DATABROKER, "Destroying DataBroker instance");

    delete wan_;
    delete local_;
}

bool DataBroker::init()
{
    logInfo(DATABROKER, "Intializing DataBroker");

    if (!enabled_)
    {
        // Setting autoenable_created_entities to false so Participants are created but they do not
        // start the discovery, and so the Listener could be initialized and get the guids without getting data
        // This avoids a deadlock in Listener
        eprosima::fastdds::dds::DomainParticipantFactoryQos qos;
        qos.entity_factory().autoenable_created_entities = false;
        if (eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->set_qos(qos) != ReturnCode_t::RETCODE_OK)
        {
            logError(DATABROKER, "Error setting autoenabled entities to false");
            return false;
        }

        if (!wan_->init())
        {
            logError(DATABROKER, "Error initializing External Participant");
            return false;
        }

        if (!local_->init())
        {
            logError(DATABROKER, "Error initializing Internal Participant");
            return false;
        }

        // Listener is initialized already with the Participants created, so it knows their guids
        listener_.init(local_, wan_);

        // Enabled Participants
        if (!wan_->enable())
        {
            logError(DATABROKER, "Error enabling External Participant");
            return false;
        }

        if (!local_->enable())
        {
            logError(DATABROKER, "Error enabling Internal Participant");
            return false;
        }

        // When initializing the DataBroker, add the topics that are already set
        for (auto topic : configuration_.active_topics)
        {
            add_topic_(topic);
        }
    }

    enabled_ = true;

    logInfo(DATABROKER, "DataBroker initialized");

    return true;
}

bool DataBroker::run()
{
    if (!enabled_)
    {
        logError(DATABROKER, "WARNING DataBroker running without being initialized");
        init();
    }

    if (configuration_.interactive)
    {
        return run_interactive();
    }
    else
    {
        return run_time(configuration_.seconds);
    }
}

bool DataBroker::run_interactive()
{
    logInfo(DATABROKER, "Running DataBroker in interactive mode");

    std::string input;
    std::vector<std::string> args;

    std::cout << "Running DataBroker in interactive mode. Write a command:" << std::endl;
    std::cout << "> " << std::flush;

    // While reading input
    while (std::getline(std::cin, input))
    {
        // Read the input and get the arguments
        switch (read_command(input, args))
        {
            case Command::ADD_TOPIC:
                std::cout << "Adding topic: " << args[0] << std::endl;
                add_topic_(args[0]);
                break;

            case Command::REMOVE_TOPIC:
                std::cout << "Removing topic: " << args[0] << std::endl;
                remove_topic_(args[0]);
                break;

            case Command::LOAD_FILE:
                std::cout << "Loading file: " << args[0] << std::endl;
                if (DataBrokerConfiguration::reload_configuration_file(configuration_, args[0]))
                {
                    stop_all_topics();
                    for (std::string topic : configuration_.active_topics)
                    {
                        add_topic_(topic);
                    }
                }
                else
                {
                    std::cout << "Error reading configuration file " << args[0] << std::endl;
                }
                break;

            case Command::RELOAD_FILE:
                std::cout << "Reloading file: " << configuration_.config_file << std::endl;
                if (DataBrokerConfiguration::reload_configuration_file(configuration_))
                {
                    stop_all_topics();
                    for (std::string topic : configuration_.active_topics)
                    {
                        add_topic_(topic);
                    }
                }
                else
                {
                    std::cout << "Error reloading configuration file " << configuration_.config_file << std::endl;
                }

                break;

            case Command::EXIT:
                return true;

            case Command::CMD_HELP:
                print_help();
                break;

            case Command::UNKNOWN:
                std::cout << "Unknown command: '" << input << "'" << std::endl;
                print_help();
                break;

            case Command::ERROR:
                std::cout << "Error in command: '" << input << "'.\n"
                    "use command 'help' to check available commands and their arguments" << std::endl;

            default:
                break;
        }
        std::cout << "> " << std::flush;
    }

    return true;
}

Command DataBroker::read_command(
        const std::string& input,
        std::vector<std::string>& args)
{
    // Get all arguments
    args.clear();
    utils::split_string(input, args, " ");

    if (args.size() < 1)
    {
        return Command::VOID;
    }

    // Get command word
    std::string command_word = args[0];
    utils::to_lowercase(command_word);
    args.erase(args.begin());

    Command command = Command::UNKNOWN;

    for (auto com : COMMAND_KEYWORDS)
    {
        if (std::find(com.second.begin(), com.second.end(), command_word) != com.second.end())
        {
            command = com.first;
            break;
        }
    }

    // Check if command should have arguments
    auto it = COMMAND_ARGUMENTS.find(command);

    if (it != COMMAND_ARGUMENTS.end())
    {
        // Should not have arguments
        if (args.size() != it->second.size())
        {
            std::cout << "Command " << command_word << " requires " << it->second.size() << " argument(s)" << std::endl;
            command = ERROR;
        }
    }

    return command;
}

void DataBroker::print_help()
{
    std::string print(
        "Reading commands from command line to interact with the running DataBroker.\n" \
        "Write a command and its arguments and press ENTER.\n" \
        "Commands are not case sensitive, but arguments are. Commands allowed :");

    // Print each of the commands, its keywords and its arguments
    for (auto command : COMMAND_KEYWORDS)
    {
        print += "\n - ";
        bool first = true;

        // Print keyword
        for (auto keyworkd : command.second)
        {
            if (first)
            {
                first = false;
            }
            else
            {
                print += " | ";
            }

            print += keyworkd;
        }

        // Print arguments
        auto it = COMMAND_ARGUMENTS.find(command.first);

        if (it != COMMAND_ARGUMENTS.end())
        {
            for (int i = 0; i < it->second.size(); ++i)
            {
                print += " <";
                print += it->second[i];
                print += ">";
            }
        }
    }

    std::cout << print << std::endl;
}

bool DataBroker::run_time(
        const uint32_t seconds)
{
    if (seconds > 0)
    {
        std::cout << "Running DataBroker for " << seconds << " seconds" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(seconds));
    }
    else
    {
        std::cout << "Running DataBroker until SIGINT is received" << std::endl;
        std::this_thread::sleep_until(std::chrono::time_point<std::chrono::system_clock>::max());
    }

    return true;
}

void DataBroker::add_topic_(
        const std::string& topic)
{
    logInfo(DATABROKER, "Adding topic '" << topic << "' to whitelist");

    topics_[topic] = true;
    // The topic needs to be unlocked in case it was locked before
    listener_.unlock_topic(topic);
    local_->add_topic(topic);
    wan_->add_topic(topic);
}

void DataBroker::remove_topic_(
        const std::string& topic)
{
    logInfo(DATABROKER, "Removing topic '" << topic << "' from whitelist");

    topics_[topic] = false;
    listener_.lock_topic(topic);
}

void DataBroker::stop_all_topics()
{
    for (auto topic : topics_)
    {
        remove_topic_(topic.first);
    }
}

} /* namespace databroker */
} /* namespace eprosima */
