# DOCKER COMPOSE TEST

This module build a ctest executable that launches different a *docker compose* files that
test a specific network architecture and scenario and check the result in different subscribers.

## Requirements

This test assumes there is an already built docker images called `$DDSROUTER_COMPOSE_TEST_DOCKER_IMAGE` (env var).
