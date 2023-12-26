#!/bin/bash
# Script to autogenerate all certificates for DDS Router Security Backdoor Test
# This script is meant to be executed from <ddsrouter_path>/ddsrouter_test/compose/test_cases/security/backdoor/generators/ (its location in the repository)

# Directory to store the files that are actually needed in test
CERTS_WORKSPACE="$(pwd)/../certs"
# Directory to store the files to configure this commands
CONF_WORKSPACE="$(pwd)/"
# Directory to store result files that are not required in test
EXTRA_WORKSPACE="$(pwd)/../extra"

mkdir -p ${CERTS_WORKSPACE}
mkdir -p ${CONF_WORKSPACE}
mkdir -p ${EXTRA_WORKSPACE}
touch ${EXTRA_WORKSPACE}/index.txt

# CA
openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -x509 -days 3650 -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -keyout ${EXTRA_WORKSPACE}/main_ca_key.pem -out ${EXTRA_WORKSPACE}/main_ca_cert.pem -config ${CONF_WORKSPACE}/main_ca_conf.cnf

# Local Pub 0
openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/local_pub0_conf.cnf -keyout ${EXTRA_WORKSPACE}/local_pub0_key.pem -out ${EXTRA_WORKSPACE}/local_pub0_req.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/main_ca_conf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/local_pub0_req.pem -out ${EXTRA_WORKSPACE}/local_pub0_cert.pem

# Local Sub 0
openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/local_sub0_conf.cnf -keyout ${EXTRA_WORKSPACE}/local_sub0_key.pem -out ${EXTRA_WORKSPACE}/local_sub0_req.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/main_ca_conf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/local_sub0_req.pem -out ${EXTRA_WORKSPACE}/local_sub0_cert.pem

# Local Pub 1
openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/local_pub1_conf.cnf -keyout ${EXTRA_WORKSPACE}/local_pub1_key.pem -out ${EXTRA_WORKSPACE}/local_pub1_req.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/main_ca_conf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/local_pub1_req.pem -out ${EXTRA_WORKSPACE}/local_pub1_cert.pem

# Local Sub 1
openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/local_sub1_conf.cnf -keyout ${EXTRA_WORKSPACE}/local_sub1_key.pem -out ${EXTRA_WORKSPACE}/local_sub1_req.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/main_ca_conf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/local_sub1_req.pem -out ${EXTRA_WORKSPACE}/local_sub1_cert.pem

# Router Local 0
openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/router_local0_conf.cnf -keyout ${EXTRA_WORKSPACE}/router_local0_key.pem -out ${EXTRA_WORKSPACE}/router_local0_req.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/main_ca_conf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/router_local0_req.pem -out ${EXTRA_WORKSPACE}/router_local0_cert.pem

# Router Local 1
openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/router_local1_conf.cnf -keyout ${EXTRA_WORKSPACE}/router_local1_key.pem -out ${EXTRA_WORKSPACE}/router_local1_req.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/main_ca_conf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/router_local1_req.pem -out ${EXTRA_WORKSPACE}/router_local1_cert.pem

# Server
openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/server_conf.cnf -keyout ${EXTRA_WORKSPACE}/server_key.pem -out ${EXTRA_WORKSPACE}/server_req.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/main_ca_conf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/server_req.pem -out ${EXTRA_WORKSPACE}/server_cert.pem

# Client
openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/client_conf.cnf -keyout ${EXTRA_WORKSPACE}/client_key.pem -out ${EXTRA_WORKSPACE}/client_req.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/main_ca_conf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/client_req.pem -out ${EXTRA_WORKSPACE}/client_cert.pem

# Governance & Permissions
openssl smime -sign -in governance.xml -text -out ${EXTRA_WORKSPACE}/governance.smime -signer ${EXTRA_WORKSPACE}/main_ca_cert.pem -inkey ${EXTRA_WORKSPACE}/main_ca_key.pem

openssl smime -sign -in permissions_lan0.xml -text -out ${EXTRA_WORKSPACE}/permissions_lan0.smime -signer ${EXTRA_WORKSPACE}/main_ca_cert.pem -inkey ${EXTRA_WORKSPACE}/main_ca_key.pem

openssl smime -sign -in permissions_lan1.xml -text -out ${EXTRA_WORKSPACE}/permissions_lan1.smime -signer ${EXTRA_WORKSPACE}/main_ca_cert.pem -inkey ${EXTRA_WORKSPACE}/main_ca_key.pem

openssl smime -sign -in permissions_wan.xml -text -out ${EXTRA_WORKSPACE}/permissions_wan.smime -signer ${EXTRA_WORKSPACE}/main_ca_cert.pem -inkey ${EXTRA_WORKSPACE}/main_ca_key.pem

#########
# Copy result files required in CERTS directory

file_list=(
    "main_ca_cert.pem"

    "local_pub0_key.pem"
    "local_pub0_cert.pem"

    "local_sub0_key.pem"
    "local_sub0_cert.pem"

    "local_pub1_key.pem"
    "local_pub1_cert.pem"

    "local_sub1_key.pem"
    "local_sub1_cert.pem"

    "router_local0_key.pem"
    "router_local0_cert.pem"

    "router_local1_key.pem"
    "router_local1_cert.pem"

    "server_key.pem"
    "server_cert.pem"

    "client_key.pem"
    "client_cert.pem"

    "governance.smime"
    "permissions_lan0.smime"
    "permissions_lan1.smime"
    "permissions_wan.smime"
)

for file in "${file_list[@]}"
do
    cp "${EXTRA_WORKSPACE}/${file}" "${CERTS_WORKSPACE}"
done
