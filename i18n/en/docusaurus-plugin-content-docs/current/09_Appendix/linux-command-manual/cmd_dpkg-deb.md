---
sidebar_position: 1
---

# dpkg-deb

The **dpkg-deb command** is a software package management tool for Debian Linux. It can perform packaging and unpackaging operations on software packages and provide package information.

## Syntax

```
  dpkg-deb [<option> ...] <command>
```

## Command Description

The dpkg-deb command not only has options to set, but also requires a command to execute different functions.

- -b: Create a Debian software package.
- -c: Display the file list in the package.
- -e: Extract the control information.
- -f: Print field contents to standard output.
- -x: Extract files from the package to the specified directory.
- -X: Extract files from the package to the specified directory and display the detailed process of extraction.
- -w: Display package information.
- -l: Display detailed package information.
- -R: Extract control information and archive manifest file.

## Option Description

- `-v, --verbose`: Enable verbose output.
- `-D, --debug`: Enable debug output.
  - `--showformat=<format>`: Use alternative format for `--show`.
  - `--deb-format=<format>`: Choose archive format. Allowed values are 0.939000, 2.0 (default).
  - `--nocheck`: Disable control file checking (build bad packages).
  - `--root-owner-group`: Force file owner and group to be root.
  - `--[no-]uniform-compression`: Use compression parameters on all members. If specified, uniform compression parameters will be used.
- `-z#`: Set compression level during build.
- `-Z<type>`: Set compression type to be used during build. Allowed types are gzip, xz, zstd, none.
- `-S<strategy>`: Set compression strategy during build. Allowed values are none, extreme (xz), filtered, huffman, rle, fixed (gzip).

### Common Commands

- Extract program files

```shell
dpkg-deb -x hobot-configs_2.2.0-20231030133209_arm64.deb
```

- Extract control files

```shell
dpkg-deb -e hobot-configs_2.2.0-20231030133209_arm64.deb hobot-configs/DEBIAN
```

- Query the content of the deb package

```shell
dpkg-deb -c hobot-configs_2.2.0-20231030133209_arm64.deb
```