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

openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -x509 -days 3650 -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -keyout ${EXTRA_WORKSPACE}/main_ca_key.pem -out ${EXTRA_WORKSPACE}/main_ca_cert.pem -config ${CONF_WORKSPACE}/main_ca_conf.cnf

openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/local_pub_conf.cnf -keyout ${EXTRA_WORKSPACE}/local_pub_key.pem -out ${EXTRA_WORKSPACE}/local_pub_req.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/main_ca_conf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/local_pub_req.pem -out ${EXTRA_WORKSPACE}/local_pub_cert.pem

openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/local_sub_conf.cnf -keyout ${EXTRA_WORKSPACE}/local_sub_key.pem -out ${EXTRA_WORKSPACE}/local_sub_req.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/main_ca_conf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/local_sub_req.pem -out ${EXTRA_WORKSPACE}/local_sub_cert.pem

openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/router_conf.cnf -keyout ${EXTRA_WORKSPACE}/router_key.pem -out ${EXTRA_WORKSPACE}/router_req.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/main_ca_conf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/router_req.pem -out ${EXTRA_WORKSPACE}/router_cert.pem

openssl smime -sign -in governance.xml -text -out ${EXTRA_WORKSPACE}/governance.smime -signer ${EXTRA_WORKSPACE}/main_ca_cert.pem -inkey ${EXTRA_WORKSPACE}/main_ca_key.pem

openssl smime -sign -in permissions.xml -text -out ${EXTRA_WORKSPACE}/permissions.smime -signer ${EXTRA_WORKSPACE}/main_ca_cert.pem -inkey ${EXTRA_WORKSPACE}/main_ca_key.pem

#########
# Copy result files required in CERTS directory

file_list=(
    "main_ca_cert.pem"
    "local_pub_key.pem"
    "local_pub_cert.pem"
    "local_sub_key.pem"
    "local_sub_cert.pem"
    "router_key.pem"
    "router_cert.pem"
    "governance.smime"
    "permissions.smime"
)

for file in "${file_list[@]}"
do
    cp "${EXTRA_WORKSPACE}/${file}" "${CERTS_WORKSPACE}"
done
