---
sidebar_position: 7
---

# 2.7 Shared File Configuration

This section introduces the usage instructions for sharing tools in Ubuntu systems.



## Samba 

### Installation Command

```bash
sudo apt install samba
```

### Configuring Samba

1. Create a shared directory. Create a directory named `shared` under your home directory to serve as the shared folder by running the following command:

```bash
mkdir ~/shared
```

2. Configure Samba sharing. Open Samba's main configuration file `/etc/samba/smb.conf` and append the following content at the end of the file to define the shared directory configuration:

```bash
[shared]
   comment = Shared Directory for Ubuntu 22.04
   path = /home/your_username/shared
   read only = no
   browsable = yes
   guest ok = no
   create mask = 0775
   directory mask = 0775
```

Syntax explanation:

```bash
[shared]: This is the name of the share. Clients will see this name when accessing shared resources; you can modify it as needed.
comment: A description of the shared directory to help users understand its purpose.
path: Specifies the actual path of the shared directory. Replace your_username with your actual username.
read only: Setting this to no allows clients to read from and write to the shared directory.
browsable: Setting this to yes makes the shared directory visible on the network.
guest ok: Setting this to no requires a username and password to access the shared directory, ensuring security.
create mask and directory mask: Set the default permissions for files and directories created within the shared directory, respectively.
```

3. Set up a Samba user and password

To access the shared directory, you need to create a Samba user and set a password. You can use an existing system user as the Samba user. Run the following command to add the system user to the Samba user list:

```bash
sudo smbpasswd -a sunrise
```

4. Restart the Samba service

```bash
sudo systemctl restart smbd
```

You can use the following command to check the status of the Samba service:

```bash
sudo systemctl status smbd
```

5. Configure the firewall (optional step)

If your system has a firewall enabled (e.g., ufw), you need to open the ports used by Samba so other devices can access the shared directory. Run the following command to allow Samba through the firewall:

```bash
sudo ufw allow samba
```



## NFS

NFS (Network File System) is a network file system that uses the classic client-server (C/S) architecture. The server manages and stores shared files and directories, while clients access these resources over the network.

This section provides a tutorial on using Ubuntu 22.04 as an NFS client.

**Prerequisites:** An NFS server must already be set up.

1. Install the NFS client software

```bash
sudo apt install nfs-common
```

2. Create a mount point

Create a local directory in Ubuntu as a mount point for the Windows NFS shared directory, for example:

```bash
sudo mkdir -p /userdata/windows_nfs_share
```

3. Mount the NFS shared directory

Use the following command to mount the Windows NFS shared directory to the Ubuntu mount point. Assuming the Windows server's IP address is `192.168.127.11` and the shared directory is `D:\NFSShare`:

```bash
sudo mount 192.168.127.11:/D:/NFSShare /userdata/windows_nfs_share
```

4. Verify the mount

Run the following command to check if the mount was successful:

```bash
df -h
```

If you see `192.168.127.11:/D:/NFSShare` mounted on `/userdata/windows_nfs_share` in the output, the mount was successful.

5. Configure automatic mounting at boot (optional)

To automatically mount the NFS shared directory every time Ubuntu boots, edit the `/etc/fstab` file:

```
sudo vi /etc/fstab
```

Add the following line at the end of the file:

```bash
192.168.127.11:/D:/NFSShare /userdata/windows_nfs_share nfs defaults 0 0
```

Save and exit the editor.