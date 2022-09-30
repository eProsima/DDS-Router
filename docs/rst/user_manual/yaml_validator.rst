.. include:: ../exports/alias.include
.. include:: ../exports/roles.include

.. _yaml_validator:

##############
YAML Validator
##############

Configuration files used to launch a DDS-Router instance need to follow a specific structure, which is extensively
described along section :ref:`user_manual_configuration`. The *YAML Validator tool* has been developed for the sole
purpose of validating user-defined configuration files in an easy manner.

.. note::

    Yaml Validator tool is supported for ``3.0`` configuration version only.

After having sourced the |ddsrouter| workspace, execute the following command in order to validate a YAML configuration
file:

.. code-block:: bash

    ddsrouter_yaml_validator --config-file ddsrouter-config.yaml

Alternatively, the user may choose to validate against a different schema, by using instead the command below:

.. code-block:: bash

    ddsrouter_yaml_validator --config-file ddsrouter-config.yaml --schema schema.json
