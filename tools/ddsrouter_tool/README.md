# eProsima DDS Router Tool

This module create an executable that runs a DDS Router configured via *yaml* configuration file.

---

## Example of usage

```sh
# Source installation first. In colcon workspace: :$ source install/setup.bash

ddsrouter --help

# Usage: Fast DDS Router
# Connect different DDS networks via DDS through LAN or WAN.
# It will build a communication bridge between the different Participants included in the provided configuration file.
# To stop the execution gracefully use SIGINT (C^) or SIGTERM (kill) signals.
# General options:
#
# Application help and information.
#   -h --help           Print this help message.
#   -v --version        Print version and commit hash.
#
# Application parameters
#   -c --config-path    Path to the Configuration File (yaml format) [Default: ./DDS_ROUTER_CONFIGURATION.yaml].
#   -r --reload-time    Time period in seconds to reload configuration file. This is needed when FileWatcher functionality is not available (e.g. config file is a symbolic link). Value 0 does not reload file. [Default: 0].
#   -t --timeout        Set a maximum time in seconds for the Router to run. Value 0 does not set maximum. [Default: 0].
#
# Debug parameters
#   -d --debug          Set log verbosity to Info (Using this option with --log-filter and/or --log-verbosity will lead to undefined behaviour).
#      --log-filter     Set a Regex Filter to filter by category or message the log entries. [Default = "DDSROUTER"].
#      --log-verbosity  Set a Log Verbosity Level higher or equal the one given. (Values accepted: "info","warning","error") [Default = "error"].
```

---

## Dependencies

* `fastcdr`
* `fastdds`
* `cpp_utils`
* `ddspipe_core`
* `ddspipe_participants`
* `ddspipe_yaml`
* `ddsrouter_core`
* `ddsrouter_yaml`
* `yaml-cpp`

Only for test:

* `python`

---
