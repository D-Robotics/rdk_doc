---
sidebar_position: 1
---

# rsync

Rsync is a fast and powerful file copying tool. It can copy files locally or from another host using any remote shell, and it can also copy files with a remote rsync daemon. It provides numerous options that allow control over various aspects of its behavior and enables flexible specification of the files to be copied. Rsync is known for its incremental transfer algorithm, which reduces the amount of data sent over the network by only sending the differences between the source and existing files in the target. Rsync is widely used for backup and mirroring operations, as well as an improved alternative to the standard copy command for everyday use.

Rsync finds files to be transferred using the default "quick check" algorithm, which looks for files that have changed in size or modification time. When the quick check indicates that the data of a file does not need to be updated, any changes to other preserved attributes on the destination file (according to the requested options) are applied directly.

## Syntax

```shell
Usage: rsync [OPTION]... SRC [SRC]... DEST
  or   rsync [OPTION]... SRC [SRC]... [USER@]HOST:DEST
  or   rsync [OPTION]... SRC [SRC]... [USER@]HOST::DEST
  or   rsync [OPTION]... SRC [SRC]... rsync://[USER@]HOST[:PORT]/DEST
  or   rsync [OPTION]... [USER@]HOST:SRC [DEST]
  or   rsync [OPTION]... [USER@]HOST::SRC [DEST]
  or   rsync [OPTION]... rsync://[USER@]HOST[:PORT]/SRC [DEST]
  
The ':' usages connect via remote shell, while '::' & 'rsync://' usages connect
to an rsync daemon, and require SRC or DEST to start with a module name.
```

Simplified syntax:

```
rsync [OPTION...] SRC... [DEST]
```

## Option Explanation

- -v, --verbose: Increase verbosity
  - --info=FLAGS: Specify info messages to output
  - --debug=FLAGS: Specify debug messages to output
  - --msgs2stderr: Special handling for debug output
- -q, --quiet: Suppress non-error messages
  - --no-motd: Suppress MOTD in daemon mode (see man page notes)
- -c, --checksum: Skip based on checksum, not modification time and size
- -a, --archive: Archive mode; equivalent to -rlptgoD (excluding -H, -A, -X)
  - --no-OPTION: Turn off implied OPTION (e.g., --no-D)
- -r, --recursive: Recurse into directories
- -R, --relative: Use relative path names
  - --no-implied-dirs: Do not send implied directories with --relative
- -b, --backup: Create backups (see --suffix and --backup-dir)
  - --backup-dir=DIR: Place backup files into a hierarchy based on DIR
  - --suffix=SUFFIX: Set the suffix for backup files (default is ~ if no --backup-dir)
- -u, --update: Skip files that are newer on the receiving end
  - --inplace: Update destination files in-place (see man page)
  - --append: Append data to the shorter file
  - --append-verify: Similar to --append, but verify the checksum of the appended file with the existing data
- -d, --dirs: Transfer directories without recursion
- -l, --links: Copy symbolic links as symbolic links
- -L, --copy-links: Convert symbolic links to the specified file/directory
  - --copy-unsafe-links: Convert "unsafe" symbolic links only
  - --safe-links: Ignore symbolic links pointing outside the source tree
  - --munge-links: Obfuscate symbolic links to make them more secure (but unusable)
- -k, --copy-dirlinks: Convert symbolic links to directories specified in the target directory
- -K, --keep-dirlinks: Treat symbolic link directories on the receiving end as directories
- -H, --hard-links: Preserve hard links
- -p, --perms: Preserve permissions
- -E, --executability: Preserve executability of files
  - --chmod=CHMOD: Affect the permissions of files and/or directories
- -A, --acls: Preserve ACLs (implies --perms)
- -X, --xattrs: Preserve extended attributes
- -o, --owner: Preserve ownership (superuser only)
- -g, --group: Preserve group ownership
  - --devices: Preserve device files (superuser only)
  - --copy-devices: Copy device contents as regular files
  - --specials: Preserve special files
- -D: Equivalent to --devices --specials
- -t, --times: Preserve modification times
- -O, --omit-dir-times: Omit directories from --times
- -J, --omit-link-times: Omit symbolic links from --times
  - --super: Receiver attempts super-user activities
  - --fake-super: Store/recover privileged attributes using xattrs
- -S, --sparse: Convert continuous empty blocks to sparse blocks
  - --preallocate: Preallocate the destination file before writing to it
- -n, --dry-run: Perform a trial run without making any changes
- -W, --whole-file: Transfer whole file (no incremental transfer algorithm used)
  - --checksum-choice=STR: Choose the checksum algorithm
- -x, --one-file-system: Do not cross filesystem boundaries
- -B, --block-size=SIZE: Force a fixed checksum block size
- -e, --rsh=COMMAND: Specify the remote shell to use
  - --rsync-path=PROGRAM: Specify the rsync to run on the remote machine
  - --existing: Skip creating new files on the receiving end
  - --ignore-existing: Skip updating files that already exist on the receiving end
  - --remove-source-files: Sender deletes synchronized files (non-directories)
  - --del: Alias for --delete-during
  - --delete: Remove files in the destination directory that are not present in the source directory
  - --delete-before: Receiver deletes files before transfer
  - --delete-during: Receiver deletes files during transfer
  - --delete-delay: Deletes files after finding the delete operation
  - --delete-after: Receiver deletes files after transfer
  - --delete-excluded: Also remove excluded files from the destination directory
  - --ignore-missing-args: Ignore missing source arguments without error
  - --delete-missing-args: Remove missing source arguments from destination
  - --ignore-errors: Continue deleting even if there are I/O errors
  - --force: Forcefully delete directories even if not empty- --max-delete=NUM: Delete at most NUM files
  - --max-size=SIZE: Do not transfer files larger than SIZE
  - --min-size=SIZE: Do not transfer files smaller than SIZE
  - --partial: Keep partially transferred files
  - --partial-dir=DIR: Put partially transferred files in DIR
  - --delay-updates: Put all updated files in a specified location at the end of the transfer
- -m, --prune-empty-dirs: Exclude empty directories from the file list
  - --numeric-ids: Don't map uid/gid values through usernames/groups
  - --usermap=STRING: Customize username mapping
  - --groupmap=STRING: Customize group name mapping
  - --chown=USER:GROUP: Simple username/group name mapping
  - --timeout=SECONDS: Set I/O timeout in seconds
  - --contimeout=SECONDS: Set timeout for daemon connection in seconds
- -I, --ignore-times: Do not skip files that match in size and modification time
- -M, --remote-option=OPTION: Only send OPTION to remote side
  - --size-only: Skip files that match in size only
- @, --modify-window=NUM: Set the accuracy of modification time comparison
- -T, --temp-dir=DIR: Create temporary files in directory DIR
- -y, --fuzzy: Find similar files as a basis if no target file exists
  - --compare-dest=DIR: Also compare target files relative to DIR
  - --copy-dest=DIR: Include copies of unchanged files
  - --link-dest=DIR: Hard link unchanged files to DIR instead of copying
- -z, --compress: Compress file data during transfer
  - --compress-level=NUM: Set compression level explicitly
  - --skip-compress=LIST: Skip compression for files with suffixes in LIST
- -C, --cvs-exclude: Auto-ignore files similar to CVS
- -f, --filter=RULE: Add a file filter rule
- -F: Same as --filter='dir-merge /.rsync-filter'
  - --exclude=PATTERN: Exclude files that match PATTERN
  - --exclude-from=FILE: Read exclude patterns from FILE
  - --include=PATTERN: Do not exclude files that match PATTERN
  - --include-from=FILE: Read include patterns from FILE
  - --files-from=FILE: Read a list of source file names from FILE
- -0, --from0: All *-from/filter files are delimited by 0
- -s, --protect-args: Do not split on spaces, only split on wildcard special characters
  - --trust-sender: Trust the file list sent by the remote sender
  - --address=ADDRESS: Bind outgoing socket to ADDRESS of daemon
  - --port=PORT: Specify an alternate PORT number for double-colon
  - --sockopts=OPTIONS: Specify custom TCP options
  - --blocking-io: Use blocking I/O for remote shell operations
  - --stats: Provide some file transfer statistics
- -8, --8-bit-output: Preserve high-bit characters in output
- -h, --human-readable: Output numbers in a human-readable format
  - --progress: Show progress during transfer
- -P: Same as --partial --progress
- -i, --itemize-changes: Output a summary of all updated changes
  - --out-format=FORMAT: Output updates using the specified FORMAT
  - --log-file=FILE: Log operations to the specified FILE
  - --log-file-format=FMT: Log updates using the specified FMT
  - --password-file=FILE: Read daemon access password from FILE- --list-only: List files without copying them
  - --bwlimit=RATE: Limit socket I/O bandwidth
  - --stop-at=y-m-dTh:m: Stop rsync at year-month-dayThour:minute
  - --time-limit=MINS: Stop rsync after MINS minutes
  - --outbuf=N|L|B: Set output buffering to none, line, or block
  - --write-batch=FILE: Write batch update to FILE
  - --only-write-batch=FILE: Similar to --write-batch, but does not update the target
  - --read-batch=FILE: Read batch update from FILE
  - --protocol=NUM: Force the use of an old protocol version
  - --iconv=CONVERT_SPEC: Request character set conversion for file names
  - --checksum-seed=NUM: Set the block/file checksum seed (advanced option)
  - --noatime: Do not change atime when opening source files
- -4, --ipv4: Use IPv4 preference
- -6, --ipv6: Use IPv6 preference
  - --version: Display the version number
- (-h) --help: Display help information (only when used alone with -h)

## Commonly Used Commands

- Copy local files: Copy files from /app directory to /userdata directory

```
rsync -avSH /app/ /userdata/
```

- Copy local machine content to remote machine

```
rsync -av /app 192.168.1.12:/app
```

- Copy remote machine content to local machine

```
rsync -av 192.168.1.12:/app /app
```

- Copy files from remote rsync server (running rsync in daemon mode) to local machine

```
rsync -av root@192.168.1.12::www /userdata
```

- Copy local machine files to remote rsync server (running rsync in daemon mode). Activate this mode when the DST path information contains the "::" separator.

```
rsync -av /userdata root@192.168.1.12::www
```

- Display file list of remote machine. This is similar to rsync transfer, but omitting the local machine information in the command.Please translate the Chinese parts in the following content into English, while keeping the original format and content:

```
rsync -v rsync://192.168.1.12/app
```

- Specify the password storage file, no need to enter the password, and directly execute rsync transfer.

```
rsync -rvzP --password-file=/etc/rsync.password rsync@$192.168.1.12::app/ /app
```