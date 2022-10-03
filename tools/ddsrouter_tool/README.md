# eProsima DDS Router Yaml Module

This module create an executable that runs a DDS Router configured via *yaml* configuration file.

---

## Example of usage

```sh
# Source installation first. In colcon workspace: :$ source install/setup.bash

ddsrouter --help

# Usage: Fast DDS Router
# Connect different DDS networks via DDS through LAN or WAN.
# It will build a communication bridge between the different Participants included in the provided configuration file.
# To close the execution gracefully use SIGINT (C^) or SIGTERM (kill).
# General options:
# -h --help         Print this help message.
# -c --config-path  Path to the Configuration File (yaml format) [Default: ./DDS_ROUTER_CONFIGURATION.yaml].
# -r --reload-time  Time period in seconds to reload configuration file. This is needed when FileWatcher functionality is not available (e.g. config file is a symbolic link).
#                     Value 0 does not reload file. [Default: 0].
# -d --debug        Activate debug Logs (be aware that some logs may require specific CMAKE compilation options).
# -v --version      Print version, branch and commit hash
```

---

## Dependencies

* `cpp_utils`
* `ddsrouter_core`
* `ddsrouter_yaml`

Only for test:

* `python`

---
