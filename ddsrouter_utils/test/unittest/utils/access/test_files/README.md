# ACCESS TEST FILES

These files are used to check the access test, that checks if
files exist and their permissions.

## Files

There are 4 different files.
Each file name refers to its permissions.

## Limitations

There will be no files without read permissions, as it will mean that it could not be copied to build,
and so it could not be tested.

## Create and change permissions to these files

Use this commands to create and change the permissions of files used for this test:

```sh
touch r__.test && chmod 444 r__.test
touch r_x.test && chmod 555 r_x.test
touch rw_.test && chmod 666 rw_.test
touch rwx.test && chmod 777 rwx.test
```
