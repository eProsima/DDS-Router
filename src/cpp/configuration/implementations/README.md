
# Configuration Implementations

These files are source C++ files (`cpp`) that implement methods `from_yaml` and `to_yaml` of
different classes.
All the implementation is included in the same directory, and not in the main classes source files
in order to not mess the source files with yaml interaction code.
Each class constructor and `dump` methods are in different files so it is not needed to compile every class
the moment one of them is needed.
