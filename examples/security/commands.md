# COMMANDS

Those have been the commands use to generate this example keys and certificates

## Priate key

```sh
openssl genrsa -des3 -out pk.key 2048
# password
```

## CA Certificate

```sh
openssl req -x509 -new -nodes -key pk.pem -sha256 -days 1825 -out ca.pem
# ES
```

## DH param

```sh
openssl dhparam -out dh.pem 2048
```
