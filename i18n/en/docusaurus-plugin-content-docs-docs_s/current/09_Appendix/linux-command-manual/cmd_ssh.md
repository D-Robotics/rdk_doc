---
sidebar_position: 1
---

# ssh

The **ssh command** is a client connection tool in the openssh suite, which can be used to securely remote login to a server using the ssh encryption protocol.

## Syntax

```
ssh [-46AaCfGgKkMNnqsTtVvXxYy] [-B bind_interface]
           [-b bind_address] [-c cipher_spec] [-D [bind_address:]port]
           [-E log_file] [-e escape_char] [-F configfile] [-I pkcs11]
           [-i identity_file] [-J [user@]host[:port]] [-L address]
           [-l login_name] [-m mac_spec] [-O ctl_cmd] [-o option] [-p port]
           [-Q query_option] [-R address] [-S ctl_path] [-W host:port]
           [-w local_tun[:remote_tun]] destination [command]
```

- **destination**: Specifies the remote ssh server to connect to.
- **command**: Specifies the command to execute on the remote ssh server.

## Option Explanation

- `-4`: Force use of IPv4 addresses.
- `-6`: Force use of IPv6 addresses.
- `-A`: Enable authentication agent connection forwarding.
- `-a`: Disable authentication agent connection forwarding.
- `-B`: Bind to `bind_interface` address before attempting to connect to the target host. Useful on systems with multiple addresses.
- `-b`: Use local specified address as the source IP address for corresponding connection.
- `-C`: Request compression of all data.
- `-F`: Specify the configuration file for SSH commands.
- `-f`: Run SSH command in the background.
- `-g`: Allow remote hosts to connect to local forwarded ports.
- `-i`: Specify an identity (private key) file.
- `-l`: Specify the login username to connect to the remote server.
- `-N`: Do not execute a remote command.
- `-o`: Specify configuration options.
- `-p`: Specify the port on the remote server.
- `-q`: Quiet mode.
- `-X`: Enable X11 forwarding.
- `-x`: Disable X11 forwarding.
- `-y`: Enable trusted X11 forwarding.

## Common Commands

```shell
# ssh username@remote_server_address
ssh sunrise@192.168.1.10
# Specify port
ssh -p 2211 sunrise@192.168.1.10

# SSH family
ssh -p 22 user@ip  # Default username is the current username, default port is 22
ssh-keygen # Generate ssh public and private keys for the current user
ssh-keygen -f keyfile -i -m key_format -e -m key_format # key_format: RFC4716/SSH2(default) PKCS8 PEM
ssh-copy-id user@ip:port # Copy the public key of the current user to the ~/.ssh/authorized_keys file on the server that needs SSH, enabling passwordless login
```

Connect to remote server

```shell
ssh username@remote_host
```

Connect to remote server and specify port

```shell
ssh -p port username@remote_host
```

Connect to remote server using a key file

```shell
ssh -i path/to/private_key username@remote_host
```

Execute a remote command locally

```shell
ssh username@remote_host "command"
```

Forward a local port to a remote server

```shell
ssh -L local_port:remote_host:remote_port username@remote_host
```

Forward a remote port to the local machine

```shell
ssh -R remote_port:local_host:local_port username@remote_host
```