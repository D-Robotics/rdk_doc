---
sidebar_position: 1
---

# mount

**mount** is a command used to mount file systems.

## Syntax

```
mount [-l|-h|-V]
mount -a [-fFnrsvw] [-t fstype] [-O optlist]
mount [-fnrsvw] [-o options] device|dir
mount [-fnrsvw] [-t fstype] [-o options] device dir
```

## Options

```shell
-V: Display the version of the program
-h: Display the help message
-v: Display verbose information, usually used for debugging with -f
-a: Mount all file systems defined in /etc/fstab
-F: This command is usually used together with -a, it creates a process for each mount action. It can speed up mounting of a large number of NFS file systems.
-f: Usually used for debugging purposes. It makes mount simulate the entire mounting process without actually performing the mount action. It is often used together with -v.
-n: By default, mount writes a record in /etc/mtab after mounting. This option cancels that action when there is no writable file system available in the system.
-s-r: Equivalent to -o ro
-w: Equivalent to -o rw
-L: Mounts a partition with a specific label.
-U: Unmounts the file system with the specified partition number. -L and -U only make sense when there is a /proc/partitions file available.
-t: Specifies the file system type. Usually not necessary, as mount automatically selects the correct type.
-o async: Enables asynchronous mode, where all file read and write operations are performed asynchronously.
-o sync: Executes in sync mode.
-o atime, -o noatime: When atime is enabled, the system updates the "last accessed time" of a file each time it is read. This option can be turned off to reduce the number of writes when using flash file systems.
-o auto, -o noauto: Enables/disables auto-mounting mode.
-o defaults: Uses default options rw, suid, dev, exec, auto, nouser, and async.
-o dev, -o nodev -o exec, -o noexec: Allows/executes execution of files.
-o suid, -o nosuid: Allows execution of files with root privileges.
-o user, -o nouser: Allows users to perform mount/umount actions.
-o remount: Remounts a file system that is already mounted with different options. For example, if the system is originally mounted as read-only, it can be remounted in read-write mode.
-o ro: Mounts in read-only mode.
-o rw: Mounts in read-write mode.
-o loop= : Uses loop mode to mount a file as a disk partition.
```

## Common Commands

Mount /dev/hda1 under /mnt:

```shell
mount /dev/hda1 /mnt
```

Mount /dev/hda1 under /mnt in read-only mode.

```
mount -o ro /dev/hda1 /mnt
```

Remount the root directory "/" in read-write mode.

```
mount -o remount,rw /
```

Mount nfs network file system.

```
mount -t nfs -o nolock 192.168.1.20:/home/ /tmp/nfs
```