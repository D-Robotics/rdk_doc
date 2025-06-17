---
sidebar_position: 1
---

# dpkg

Install, create, and manage software packages on Debian Linux systems.

The **dpkg command** is a utility tool used on Debian Linux systems to install, create, and manage software packages.

## Syntax

```
dpkg [<option> ...] <command>
```

## Command explanations

The dpkg command has options for setting and requires a command to execute different functionalities.

- -i: Install a software package.
- -r: Remove a software package.
- -P: Remove a software package and its configuration files.
- -L: List files belonging to a specified software package.
- -l: List software package status concisely.
- -S: Search for software packages containing specified files.
- --unpack: Unpack a software package.
- -c: Show the list of files inside a software package.
- --configure: Configure a software package.

## Option explanations

- --admindir= directory: Use directory instead of /var/lib/dpkg.
- --root= directory: Install to a different root directory.
- --instdir=directory: Change the installation directory while maintaining the management directory.
- --path-exclude= expression: Do not install paths that match the shell expression.
- --path-include= expression: Include an additional pattern after exclusion patterns.
- -O|--selected-only: Ignore software packages that are not selected for installation or upgrade.
- -E|--skip-same-version: Ignore software packages with the same version as the installed package.
- -G|--refuse-downgrade: Ignore software packages with versions earlier than the installed package.
- -B|--auto-deconfigure: Install even if it affects other software packages.
- --[no-]triggers: Skip or force the handling of triggers that follow.
- --verify-format=format: Check the output format ('rpm' is supported).
- --no-debsig: Do not attempt to verify the signature of the software package.
- -D|--debug=octal: Enable debugging (see -Dhelp or --debug=help for more information).
- --status-logger=command: Send status updates to the standard input of command.
- --log=filename: Log status updates and operation information to filename.
- --ignore-depends=package,...: Ignore all dependencies related to package.
- --force-...: Ignore encountered problems (see --force-help for more details).
- --no-force-...|--refuse-...: Abort execution when encountering problems.
- --abort-after `<n>`         Abort after encountering `<n>` errors.

## Common Commands

- Install package

```
dpkg -i package.deb
```

- Remove package

```
dpkg -r package
```

- Remove package (including configuration files)

```
dpkg -P package
```

- List the files associated with the package

```
dpkg -L package
```

- Show the version of the package

```
dpkg -l package
```

- Unpack the contents of a deb package

```
dpkg --unpack package.deb
```

- Search for the package that owns the specified file or keyword

```
dpkg -S keyword
```

- List all currently installed packages

```
dpkg -l
```

- List the contents of a deb package

```
dpkg -c package.deb
```

- Configure the package

```
dpkg --configure package
```
