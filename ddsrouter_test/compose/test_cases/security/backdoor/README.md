# DDS ROUTER SECURITY BACKDOOR TEST

This DDS-Router test belongs to the docker tests that are executed using a docker-compose application that rises several DDS Participants and DDS Routers.
This test case check the connection between a secured and a non secured DDS Domain.

## Test scenario

In this test case, there are 2 DDS Domains and 1 DDS Router:

- **A**: DomainParticipants belong to DDS Domain 0.
  - Entities:
    - DomainParticipant: DataWriter [topic_0]
    - DomainParticipant: DataReader [topic_1]
  - Executable:
    - `AdvancedConfigurationExample`

- **S**: DomainParticipants belong to DDS Domain 0 and uses DDS Security with authentication and encryption.
  - Entities:
    - DomainParticipant: DataReader [topic_0]
    - DomainParticipant: DataWriter [topic_1]
  - Configuration
    - XML:
      - `configurations/secure_participant_configuration.xml`
      - profile name: `xml_secure_participant_backdoor_test`
  - Executable:
    - `AdvancedConfigurationExample`

- **R**: DDS Router that connect both participants
  - Entities:
    - Participant `local`
    - Participant `xml`
  - Configuration:
    - XML:
      - `configurations/router_configuration.xml`
      - profile name: `xml_secure_router_backdoor_test`
    - yaml:
      - `ddsrouter.yaml`
  - Executable:
    - `ddsrouter`

### Topics

All topics are best-effort volatile.
All topics are allowed for both Domain *S* and DDS Router

Thus, the expected behavior is:

- *A* DataReader [topic_1]: Read something
- *S* DataReader [topic_0]: Read something
