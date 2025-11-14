---
sidebar_position: 1
---

# scp

The Linux scp command is used to copy files and directories between Linux systems.

scp is short for secure copy. It is a secure remote file copy command based on ssh login in the Linux system.

scp is encrypted, while [rcp](https://www.runoob.com/linux/linux-comm-rcp.html) is not encrypted. scp is an enhanced version of rcp.

## Syntax

```
scp [-346BCpqrTv] [-c cipher] [-F ssh_config] [-i identity_file]
            [-J destination] [-l limit] [-o ssh_option] [-P port]
            [-S program] source ... target
```

Simplified form:

```
scp [option] file_source file_target
```

- **file_source**: Specifies the source file to be copied.
- **file_target**: The target file. The format is `user@host:filename` (filename is the name of the target file).

## Option Description

- -3: Transfers files between two remote hosts through the local host. If this option is not used, data will be transferred directly between the two remote hosts. Note that this option disables the progress display during transfer.
- -4: Forces scp to use IPv4 addressing only.
- -6: Forces scp to use IPv6 addressing only.
- -B: Batch mode. Prevents asking for a password or passphrase.
- -C: Enables compression. (Passes the -C flag to ssh, which opens the compression feature)
- -p: Preserves the modification time, access time, and access permissions of the original file.
- -q: Quiet mode. Disables the progress meter and warning and diagnostic messages from ssh(1).
- -r: Recursively copies the entire directory. Note that scp will follow symbolic links encountered in directory traversal.
- -T: Disables strict filename checking. By default, when copying files from a remote host to a local directory, scp checks that the received filenames match the requested filenames on the command line to prevent unexpected or unnecessary files from being sent by the remote side. These checks may cause desired files to be rejected due to different ways of filename wildcard interpretation by different operating systems and shells. This option disables these checks but requires complete trust in the server not to send unexpected filenames.
- -v: Verbose mode. Causes scp and ssh(1) to print debugging messages about their progress. This is helpful for debugging connection, authentication, and configuration problems.
- -c cipher: Specifies the password used for encryption during data transmission. This option is passed directly to ssh(1).
- -F ssh_config: Specifies an alternative per-user configuration file to be used instead of the default ssh(1) configuration file. This option is passed directly to ssh(1).
- -i identity_file: Specifies the identity (private key) file to be used for public key authentication. This option is passed directly to ssh(1).
- -l limit: Limits the bandwidth used, in Kbit/s.
- -o ssh_option: Passes options in the format used in ssh_config(5) to ssh. This is useful for specifying options that do not have separate scp command-line flags.
- -P port: Specifies the port to connect to on the remote host. Note that this option uses uppercase 'P' because lowercase '-p' is already reserved for preserving the modification time and mode of files.
- -S program: Specifies the program to use for the encrypted connection. The program must understand ssh(1) options.

## Common Commands

**Copying from local to remote**

Command format:

```
scp local_file remote_username@remote_ip:remote_folder 
or 
scp local_file remote_username@remote_ip:remote_file 
or 
scp local_file remote_ip:remote_folder 
or 
scp local_file remote_ip:remote_file 
```

- The first and second ones specify the username. After executing the command, you will need to enter the password. The first one specifies only the remote directory and keeps the file name the same, while the second one specifies the file name.
- The third and fourth ones do not specify the username. After executing the command, you will need to enter the username and password. The third one specifies only the remote directory and keeps the file name the same, while the fourth one specifies the file name.

Example:

```
scp /home/sunrise/test.c root@192.168.1.10:/userdata 
scp /home/sunrise/test.c root@192.168.1.10:/userdata/test_01.c
scp /home/sunrise/test.c 192.168.1.10:/userdata
scp /home/sunrise/test.c 192.168.1.10:/userdata/test_01.c
```

Copying directory command format:

```
scp -r local_folder remote_username@remote_ip:remote_folder 
or 
scp -r local_folder remote_ip:remote_folder 
```

- The first one specifies the username. After executing the command, you will need to enter the password.
- The second one does not specify the username. After executing the command, you will need to enter the username and password.

Example:

```
scp -r /home/sunrise/app/ root@192.168.1.10:/userdata/app/ 
scp -r /home/sunrise/app/ 192.168.1.10:/userdata/app/ 
```

The above commands will copy the local `app` directory to the remote `/userdata/app/` directory.

**Copying from remote to local**

The scp command for copying from remote to local is similar to the above command. Just swap the order of the last two parameters in the command for copying from local to remote.Copy files from a remote machine to a local directory

```shell
scp sunrise@192.168.1.10:/userdata/log.log /home/sunrise/
```

Download the file `log.log` from the directory `/userdata/` on the machine with IP address 192.168.1.10 to the local directory `/home/sunrise/`.