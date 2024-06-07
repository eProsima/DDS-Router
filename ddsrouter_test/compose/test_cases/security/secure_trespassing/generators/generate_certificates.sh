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

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/local_pub1_conf.cnf -keyout ${EXTRA_WORKSPACE}/local_pub1_key.pem -out ${EXTRA_WORKSPACE}/local_pub1_req.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/main_ca_conf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/local_pub1_req.pem -out ${EXTRA_WORKSPACE}/local_pub1_cert.pem

openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/local_pub3_conf.cnf -keyout ${EXTRA_WORKSPACE}/local_pub3_key.pem -out ${EXTRA_WORKSPACE}/local_pub3_req.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/main_ca_conf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/local_pub3_req.pem -out ${EXTRA_WORKSPACE}/local_pub3_cert.pem

openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/local_pub4_conf.cnf -keyout ${EXTRA_WORKSPACE}/local_pub4_key.pem -out ${EXTRA_WORKSPACE}/local_pub4_req.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/main_ca_conf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/local_pub4_req.pem -out ${EXTRA_WORKSPACE}/local_pub4_cert.pem

openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/local_pub5_conf.cnf -keyout ${EXTRA_WORKSPACE}/local_pub5_key.pem -out ${EXTRA_WORKSPACE}/local_pub5_req.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/main_ca_conf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/local_pub5_req.pem -out ${EXTRA_WORKSPACE}/local_pub5_cert.pem

openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/local_sub0_conf.cnf -keyout ${EXTRA_WORKSPACE}/local_sub0_key.pem -out ${EXTRA_WORKSPACE}/local_sub0_req.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/main_ca_conf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/local_sub0_req.pem -out ${EXTRA_WORKSPACE}/local_sub0_cert.pem

openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/local_sub2_conf.cnf -keyout ${EXTRA_WORKSPACE}/local_sub2_key.pem -out ${EXTRA_WORKSPACE}/local_sub2_req.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/main_ca_conf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/local_sub2_req.pem -out ${EXTRA_WORKSPACE}/local_sub2_cert.pem

openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/local_sub3_conf.cnf -keyout ${EXTRA_WORKSPACE}/local_sub3_key.pem -out ${EXTRA_WORKSPACE}/local_sub3_req.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/main_ca_conf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/local_sub3_req.pem -out ${EXTRA_WORKSPACE}/local_sub3_cert.pem

openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/local_sub4_conf.cnf -keyout ${EXTRA_WORKSPACE}/local_sub4_key.pem -out ${EXTRA_WORKSPACE}/local_sub4_req.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/main_ca_conf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/local_sub4_req.pem -out ${EXTRA_WORKSPACE}/local_sub4_cert.pem

openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/local_sub5_conf.cnf -keyout ${EXTRA_WORKSPACE}/local_sub5_key.pem -out ${EXTRA_WORKSPACE}/local_sub5_req.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/main_ca_conf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/local_sub5_req.pem -out ${EXTRA_WORKSPACE}/local_sub5_cert.pem

openssl ecparam -name prime256v1 > ${EXTRA_WORKSPACE}/ecdsaparam

openssl req -nodes -new -newkey ec:${EXTRA_WORKSPACE}/ecdsaparam -config ${CONF_WORKSPACE}/router_conf.cnf -keyout ${EXTRA_WORKSPACE}/router_key.pem -out ${EXTRA_WORKSPACE}/router_req.pem

openssl ca -batch -create_serial -config ${CONF_WORKSPACE}/main_ca_conf.cnf -days 3650 -in ${EXTRA_WORKSPACE}/router_req.pem -out ${EXTRA_WORKSPACE}/router_cert.pem

openssl smime -sign -in governance.xml -text -out ${EXTRA_WORKSPACE}/governance.smime -signer ${EXTRA_WORKSPACE}/main_ca_cert.pem -inkey ${EXTRA_WORKSPACE}/main_ca_key.pem

openssl smime -sign -in permissions.xml -text -out ${EXTRA_WORKSPACE}/permissions.smime -signer ${EXTRA_WORKSPACE}/main_ca_cert.pem -inkey ${EXTRA_WORKSPACE}/main_ca_key.pem

#########
# Copy result files required in CERTS directory

file_list=(
    "main_ca_cert.pem"
    "local_pub1_key.pem"
    "local_pub1_cert.pem"
    "local_pub3_key.pem"
    "local_pub3_cert.pem"
    "local_pub4_key.pem"
    "local_pub4_cert.pem"
    "local_pub5_key.pem"
    "local_pub5_cert.pem"
    "local_sub0_key.pem"
    "local_sub0_cert.pem"
    "local_sub2_key.pem"
    "local_sub2_cert.pem"
    "local_sub3_key.pem"
    "local_sub3_cert.pem"
    "local_sub4_key.pem"
    "local_sub4_cert.pem"
    "local_sub5_key.pem"
    "local_sub5_cert.pem"
    "router_key.pem"
    "router_cert.pem"
    "governance.smime"
    "permissions.smime"
)

for file in "${file_list[@]}"
do
    cp "${EXTRA_WORKSPACE}/${file}" "${CERTS_WORKSPACE}"
done
