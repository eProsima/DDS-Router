.. include:: ../../exports/alias.include

.. _user_manual_participants_xml:

###############
XML Participant
###############

This type of :term:`Participant` refers to a :term:`DomainParticipant` that uses QoS profiles loaded from XML files to be configured.

|fastdds| supports XML to fully configure a DomainParticipant.
Using XML configuration, user have whole access to the full configuration of a DDS DomainParticipant.
Check following `documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/domainparticipant.html>` for further information about configuring a DDS DomainParticipant with XML.
For further information regarding how to load XML configuration files to |ddsrouter| check following section :ref:`user_manual_configuration_load_xml`.

.. note::

    This kind of Participant is meant for advanced users as XML profiles will overwrite the default internal settings of the DDS Router.


Use case
========

Use this Participant to fully configure a DomainParticipant, its discovery methods, transport options, DDS QoS, etc.
Main use case for this Participant is using **DDS Security**, that requires XML configuration from user side.

.. TODO

    Add link to security documentation

.. warning::

    This Participant kind does not support :term:`RPC`.
    Thus services and actions of ROS 2 will not work correctly.


Kind aliases
============

* ``xml``
* ``XML``


Configuration
=============

XML Participant allows setting a profile name for the internal DomainParticipant of the |ddsrouter|.
Such name will be used as QoS profile when creating the internal DomainParticipant.


.. _user_manual_participants_xml_profiles:

Create a Fast DDS XML Participant profile
-----------------------------------------

Whole DomainParticipant configuration settings must be configured via XML, |ddsrouter| will not configure any attribute or QoS for it.
To configure the profile, check the following section: :ref:`Configuration section <user_manual_configuration_profile>`.

However, there are specific QoS that will affect the performance of the |ddsrouter| and that are advisable for the user to set them.
Notice that not setting such QoS will not affect the correct functionality of the application, but may affect its performance.

* ``ignore_local_endpoints`` avoid local matching for this participant's endpoints:

    .. code-block:: xml

        <participant profile_name="ignore_local_endpoints_domainparticipant_xml_profile">
            <rtps>
                <propertiesPolicy>
                    <properties>
                        <property>
                            <name>fastdds.ignore_local_endpoints</name>
                            <value>true</value>
                        </property>
                    </properties>
                </propertiesPolicy>
            </rtps>
        </participant>


Configuration Example
=====================

Configure a XML Participant that gets all of its QoS from XML profile named ``custom_participant_configuration``.
This XML profile must be previously loaded.
Use |fastdds| or |ddsrouter| support to load XML configuration files as explained in :ref:`this section <user_manual_configuration_load_xml>`.

.. code-block:: yaml

    - name: xml_participant                       # Participant Name = xml_participant

      kind: xml

      profile: custom_participant_configuration   # Configure participant with this profile
