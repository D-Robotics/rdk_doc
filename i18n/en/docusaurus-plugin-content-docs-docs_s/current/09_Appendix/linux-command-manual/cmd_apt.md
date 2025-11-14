---
sidebar_position: 1
---

# apt

apt (Advanced Packaging Tool) is a shell front-end package manager in Debian and Ubuntu.

The apt command provides commands to search, install, upgrade, and remove one, multiple, or all software packages. The commands are concise and easy to remember.

The apt command execution requires superuser (root) privileges.

## Syntax

```
  apt [options] [command] [package ...]
```

- **options:** Optional, options include -h (help), -y (answer "yes" to all prompts during installation), -q (do not display installation progress), and so on.
- **command:** The operation to be performed.
- **package:** The package name to be installed.

## Common commands

- Update the apt software source database: **sudo apt update**

- Update installed software packages: **sudo apt upgrade**

  List upgradeable software packages and their versions: **apt list --upgradeable**

  Upgrade software packages, removing any that need to be updated: **sudo apt full-upgrade**

- Install a specified software package: **sudo apt install `<package_name>`**

  Install multiple software packages: **sudo apt install `<package_1> <package_2> <package_3>`**

- Display detailed information about a software package, such as its version, installation size, dependencies, etc: **sudo apt show `<package_name>`**

- Remove a software package: **sudo apt remove `<package_name>`**

- Clean up unused dependencies and library files: **sudo apt autoremove**

- Remove a software package and its configuration files: **sudo apt purge `<package_name>`**

- Search for a software package: **sudo apt search `<package_name>`**

- List all installed packages: **apt list --installed**

- List version information of all installed packages: **apt list --all-versions**