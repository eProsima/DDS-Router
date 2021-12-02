.. include:: ../../exports/alias.include

.. _user_manual_participant:

######################
DDS Router Participant
######################

*DDS Router Participant* is a |ddsrouter| entity that works as an interface between a network and
the core of the router.
:term:`Participants<Participant>` are the main elements inside the |ddsrouter| functionality.

.. contents::
    :local:
    :backlinks: none
    :depth: 2

.. _user_manual_participant_participant:

Participant
===========

A :term:`Participant` is an abstraction over the DDS :term:`DomainParticipant`.
This entity manages the dynamic discovery of DDS entities on a specific network or interface.
Each :term:`Participant` is uniquely identified by a :term:`Participant Id` in a |ddsrouter| execution and has a
predefined :term:`Participant Type` that specifies the internal general functionality of the :term:`Participant`.

.. _user_manual_participant_participant_id:

Participant Id
--------------

It is an alphanumeric string that uniquely identifies a :term:`Participant` in a |ddsrouter| execution.

.. _user_manual_participant_participant_type:

Participant Type
----------------

It specifies the kind of the :term:`Participant`.
There are several Participant types already defined, which will specify in general terms how the
:term:`Participant` behaves.

Participant creation
====================

Each :term:`Participant` has a unique :term:`Participant Id` that can not be repeated in a |ddsrouter| execution.
This id is the name of the tag that contains the :term:`Participant` configuration.

.. note:

    If the id is repeated, the yaml will be bad formed and the |ddsrouter| execution will fail when configured.

Each :term:`Participant Type` is associated with one or several names or aliases that represent it.
In order to use a :term:`Participant` of a specific type, use `type` tag in the yaml configuration file, or set the
:term:`Participant Id` as the alias of the type.
If the type is not any of the valid aliases, the :term:`Participant` will not be created and the
execution will fail.

There could be as many :term:`Participants<Participant>` as required, and their types could be repeated,
but all ids must be unique.

Below are some examples of how to configure a :term:`Participant`:

.. code-block:: yaml

    participant_1:     # New Participant with Id: 'participant_1'
        type: simple   # 'participant_1' will be created of type 'simple'
        extra_configuration: ...

.. code-block:: yaml

    simple:     # New Participant with Id: 'simple' and Type: 'simple'
        extra_configuration: ...

Participant types
=================

Below is the list with all the available Participant Types.

.. list-table::
    :header-rows: 1

    *   - Participant Type
        - Aliases
        - Specific configuration tags
        - Description
    *   - :ref:`user_manual_participants_echo`
        - ``echo``
        -
        - Print in `stdout` every data received.

..
    This toctree is needed so participants files are linked from somewhere. It is hidden so it is not be visible.

.. toctree::
    :hidden:

    echo
