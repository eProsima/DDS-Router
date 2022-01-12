.. include:: ../../exports/alias.include
.. include:: ../../exports/roles.include

.. _user_manual_participant:

######################
DDS Router Participant
######################

DDS Router :term:`Participant` is a |ddsrouter| entity that works as an interface between a network and
the core of the router.
Participants are the main elements inside the |ddsrouter| functionality.

.. contents::
    :local:
    :backlinks: none
    :depth: 2

.. _user_manual_participant_participant:

Participant
===========

A Participant is an abstraction over the DDS :term:`DomainParticipant`.
This entity manages the dynamic discovery of DDS entities on a specific network or interface.
Each Participant is uniquely identified by a :term:`Participant Id` in a |ddsrouter| execution and has a
predefined :term:`Participant Type` that specifies the internal general functionality of the Participant.

.. _user_manual_participant_participant_id:

Participant Id
--------------

It is an alphanumeric string that uniquely identifies a Participant in a |ddsrouter| execution.

.. _user_manual_participant_participant_type:

Participant Type
----------------

It specifies the kind of the Participant.
There are several Participant types already defined, which will specify in general terms how the
Participant behaves.

Participant creation
====================

Each Participant has a unique Participant Id that can not be repeated in a |ddsrouter| execution.
This id is the name of the tag that contains the Participant configuration.

.. note::

    If the id is repeated, the yaml will be bad formed and the |ddsrouter| execution will fail when configured.

Each Participant Type is associated with one or several names or aliases that represent it.
In order to use a Participant of a specific type, use ``type`` tag in the yaml configuration file, or set the
Participant Id as the alias of the type.
If the type is not any of the valid aliases, the Participant will not be created and the
execution will fail.

.. note::

    There could be as many Participants as required, and their types could be repeated,
    but all ids must be unique.

Below are some examples on how to configure a Participant:

.. code-block:: yaml

    participant_1:     # New Participant with Id: 'participant_1'
      type: simple     # 'participant_1' will be created of type 'simple'
      extra_configuration: ...

.. code-block:: yaml

    simple:     # New Participant with Id: 'simple' and Type: 'simple'
      extra_configuration: ...

.. _user_manual_participant_participant_types:

Participant types
=================

Below is the list with all the available Participant Types.

.. list-table::
    :header-rows: 1

    *   - Participant Type
        - Aliases
        - Specific |br|
          configuration tags
        - Description

    *   - :ref:`user_manual_participants_echo`
        - ``echo``
        -
        - Print in `stdout` every data received.

    *   - :ref:`user_manual_participants_simple`
        - ``simple`` |br|
          ``local``
        - ``domain``
        - Simple DDS DomainParticipant.

    *   - :ref:`user_manual_participants_local_discovery_server`
        - ``discovery-server`` |br|
          ``local-ds`` |br|
          ``ds``
        - ``guid`` |br|
          ``listening-addresses`` |br|
          ``connection-addresses``
        - Discovery Server DDS DomainParticipant |br|
          for local communication.

    *   - :ref:`user_manual_participants_wan`
        - ``wan`` |br|
          ``router``
        - ``guid`` |br|
          ``listening-addresses`` |br|
          ``connection-addresses``
        - Discovery Server DDS DomainParticipant |br|
          for WAN communication.


..
    This toctree is needed so participants files are linked from somewhere. It is hidden so it is not be visible.

.. toctree::
    :hidden:

    echo
    simple
    local_discovery_server
    wan
