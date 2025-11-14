---
sidebar_position: 1
---

# tar

The **tar command** can be used to create archives for files and directories in Linux. With tar, you can create archives (backup files) for specific files, modify files within an archive, or add new files to an archive. Tar was originally used to create archives on tapes, but now users can create archives on any device. Using the tar command, you can pack a large number of files and directories into a single file, which is very useful for backing up files or combining multiple files into one file for network transmission.

First, let's clarify two concepts: packing and compressing. Packing refers to turning a large number of files or directories into a single file, while compressing means using compression algorithms to turn a large file into a small file.

Why do we need to distinguish between these two concepts? This is because many compression programs in Linux can only compress one file at a time. So when you want to compress a large number of files, you need to first pack these files into one package (using the tar command), and then compress the package using a compression program (gzip or bzip2 command).

## Syntax

```
tar [OPTION...] [FILE]...
```

## Option Description

```
-A, --catenate, --concatenate   append tar files to an archive
-c, --create               create a new archive
-d, --diff, --compare      find differences between archive and file system
    --delete               delete from the archive (not on tape!)
-r, --append               append files to the end of the archive
-t, --list                 list the contents of an archive
    --test-label           test the volume label and exit
-u, --update               only append files that are newer than the copy
-x, --extract, --get       extract files from an archive

Operation modifiers:

      --check-device         check device numbers when creating incremental archives (default)
  -g, --listed-incremental=FILE   handle new-style GNU format incremental backups
  -G, --incremental          handle old-style GNU format incremental backups
      --ignore-failed-read   do not exit with nonzero on unreadable files
      --level=NUMBER         output level for listing incremental archives created with --incremental
  -n, --seek                 archive can be seeked
      --no-check-device      do not check device numbers when creating incremental archives
      --no-seek              archive cannot be seeked
      --occurrence[=NUMBER]  process only the NUMBERth occurrence of each file in the archive;
                             this option is only valid when used in combination with one of the
                             following subcommands: --delete, --diff, --extract, or --list;
                             and regardless of whether the file list is given in command
                             line form or specified via -T option; NUMBER defaults to 1
      --sparse-version=MAJOR[.MINOR]
                             set version of the sparse format to be used (implied by --sparse)
-S, --sparse               Efficiently handle sparse files

Rewrite control:

-k, --keep-old-files       Don't replace existing files when extracting,
                            treat them as errors
    --keep-directory-symlink   Preserve existing symlinks to directories when
                            extracting
    --keep-newer-files
                            Do not replace existing files that are newer than the copy in the archive
    --no-overwrite-dir     Preserve metadata of existing directories
    --overwrite            Overwrite existing files when extracting
    --overwrite-dir        Overwrite metadata of existing directories when extracting (default)

    --recursive-unlink     Remove directory hierarchy before extracting directories
    --remove-files         Delete files after adding them to the archive
    --skip-old-files       Don't replace existing files when extracting,
                            silently skip over them
-U, --unlink-first         Delete files that would be overwritten before extracting
-W, --verify               Attempt to verify the archive after writing

Output streams:

    --ignore-command-error Ignore the exit code of subprocesses
    --no-ignore-command-error
                            Treat non-zero exit code of subprocesses as error
-O, --to-stdout            Extract files to standard output
    --to-command=COMMAND  Pipe extracted files to another program

File attributes:

    --atime-preserve[=METHOD]
                            Preserve access time on output files, either by restoring timestamps after reading (default METHOD='replace') or not setting them at all on the first (METHOD='system') setting
    --delay-directory-restore
                            Set modification time and directory permissions only after extraction is complete
    --group=NAME          Force NAME to be the group owner of the files being added
    --mode=CHANGES         Force the (symbolic) mode of the files being added to be CHANGES
    --mtime=DATE-OR-FILE   Set the mtime of the added files from DATE-OR-FILE
-m, --touch                Do not extract modification time of files
    --no-delay-directory-restore
                            Undo the effect of --delay-directory-restore option
    --no-same-owner
                            Extract files as yourself (default for regular users)
    --no-same-permissions
                            Use the user's umask when extracting permissions from the archive (default for regular users)--numeric-owner Always use numeric IDs for user/group names
    --owner=NAME Force NAME to be the owner of the added files

-p, --preserve-permissions, --same-permissions Preserve file permissions during extraction (default for super user only)
    --preserve Same as -p and -s
    --same-owner Try to preserve owner relationship during extraction (default for super user)

-s, --preserve-order, --same-order member arguments are listed in the same order as the files in the archive

Handling of extended file attributes:

    --acls Enable POSIX ACLs support
    --no-acls Disable POSIX ACLs support
    --no-selinux Disable SELinux context support
    --no-xattrs Disable extended attributes support
    --selinux Enable SELinux context support
    --xattrs Enable extended attributes support
    --xattrs-exclude=MASK Specify the exclude pattern for xattr keys
    --xattrs-include=MASK Specify the include pattern for xattr keys

Device selection and switching:

-f, --file=ARCHIVE Use archive file or device ARCHIVE
    --force-local Treat the archive file as a local archive even if it has a copy

-F, --info-script=NAME, --new-volume-script=NAME Run script at end of each tape volume (implied -M)

-L, --tape-length=NUMBER Change tape after writing NUMBER × 1024 bytes

-M, --multi-volume Create/list/extract multiple-volume archive files
    --rmt-command=COMMAND Use the specified rmt COMMAND instead of rmt
    --rsh-command=COMMAND Use the specified remote COMMAND instead of rsh
    --volno-file=FILE Use/update volume number in FILE

Device blocking:

-b, --blocking-factor=BLOCKS Number of records per BLOCKS × 512 bytes
-B, --read-full-records Reblock as reading (only for 4.2BSD pipes)
-i, --ignore-zeros Ignore zero-byte blocks in the archive (end of file)
    --record-size=NUMBER Number of bytes per record NUMBER × 512

Selecting archive format:

-H, --format=FORMAT Create archive in the specified FORMAT

FORMAT can be one of the following:

    gnu                      GNU tar 1.13.x format
    oldgnu                   GNU format as per tar <= 1.12
    pax                      POSIX 1003.1-2001 (pax) format
    posix                    equivalent to pax
    ustar                    POSIX 1003.1-1988 (ustar) format
    v7                       old V7 tar format

    --old-archive, --portability
                                equivalent to --format=v7
    --pax-option=keyword[[:]=value][,keyword[[:]=value]]...
                                control pax keywords
    --posix                equivalent to --format=posix

-V, --label=TEXT           create archive with volume name TEXT
                            use TEXT as a pattern for list/extract

Compression options:

-a, --auto-compress        use archive suffix to determine the compression program
-I, --use-compress-program=PROG
                             filter through program PROG (must accept -d option)
-j, --bzip2                filter the archive through bzip2
-J, --xz                   filter the archive through xz
    --lzip                 filter the archive through lzip
    --lzma                 filter the archive through lzma
    --lzop
    --no-auto-compress     do not use archive suffix to determine the compression program
-z, --gzip, --gunzip, --ungzip   filter the archive through gzip
-Z, --compress, --uncompress   filter the archive through compress

Local file selection:

    --add-file=FILE        add specified FILE to archive (useful if name begins with -)
    --backup[=CONTROL]     backup before removal, choose version CONTROL
-C, --directory=DIR        change to directory DIR
    --exclude=PATTERN      exclude files that match PATTERN
    --exclude-backups      exclude backups and lock files
    --exclude-caches       exclude contents of directories containing CACHEDIR.TAG, except the tag file itself
    --exclude-caches-all   exclude directories containing CACHEDIR.TAG
    --exclude-caches-under exclude all contents of directories containing CACHEDIR.TAG
    --exclude-tag=FILE     exclude contents of directories containing FILE, except the file itself
    --exclude-tag-all=FILE exclude directories containing FILE
    --exclude-tag-under=FILE   exclude all contents of directories containing FILE
    --exclude-vcs          exclude version control system directories

-h, --dereference           follow symlinks; archive and output files they point to
    --hard-dereference      Track hard links; archive and output files they point to
    
-K, --starting-file=MEMBER-NAME
                            Begin at member MEMBER-NAME when reading the
                            archive
    --newer-mtime=DATE     Compare data and time only when data has changed
    --no-null              Disable the effect of previous --null option
    --no-recursion         Avoid automatic descent in directories
    --no-unquote           Do not quote file names read with -T as ending quotes
    --null                 Read null-terminated names from -T, disable -C
-N, --newer=DATE-OR-FILE, --after-date=DATE-OR-FILE
                            Only save files that are newer than DATE-OR-FILE
    --one-file-system      Create archive while staying on local file system
-P, --absolute-names       Do not strip leading '/' from file names
    --recursion            Directory recursion (default)
    --suffix=STRING        Backup before removal, overwrite common suffix ('')
                            unless overridden by SIMPLE_BACKUP_SUFFIX
-T, --files-from=FILE      Extract or create files using names from FILE
    --unquote              Use file names read with -T as ending quotes (default)
-X, --exclude-from=FILE    Exclude patterns listed in FILE

 File name transformations:

    --strip-components=NUMBER   Remove NUMBER leading components from file names when extracting
    --transform=EXPRESSION, --xform=EXPRESSION
                            Use sed expression instead of EXPRESSION
                            for transforming file names

 File name matching options (affect both exclude and include patterns):

    --anchored             Pattern matches file name at the start
    --ignore-case          Ignore case when matching
    --no-anchored          Pattern matches anything after '/'
                            (default for exclusion)
    --no-ignore-case       Case-sensitive matching (default)
    --no-wildcards         Match strings literally
    --no-wildcards-match-slash   Wildcards do not match '/'
    --wildcards            Use wildcards (default)
    --wildcards-match-slash   Wildcards match '/'

 Informative output:

    --checkpoint[=NUMBER]  Show progress information every NUMBER records
                            (default: 10)
    --checkpoint-action=ACTION   Execute ACTION at each checkpoint--full-time                    print file time to its full resolution
    --index-file=FILE              send detailed output to FILE
-l, --check-links              print information as long as not all links are output
    --no-quote-chars=STRING        disable character quoting from STRING
    --quote-chars=STRING           additional quoting characters from STRING
    --quoting-style=STYLE          set the name quoting style; valid STYLE values are listed below
-R, --block-number             display the number of blocks in each message of the archive
    --show-defaults                display tar default options
    --show-omitted-dirs            list directories that don't match the search criteria when listing or extracting
    --show-transformed-names, --show-stored-names     display transformed file or archive names
    --totals[=SIGNAL]              print total byte count after processing the archive; when this SIGNAL is triggered with an argument of '-', print total byte count; allowed signals are: SIGHUP, SIGQUIT, SIGINT, SIGUSR1, and SIGUSR2; signals without the 'SIG' prefix are also accepted
    --utc                          print file modification time in UTC format
-v, --verbose                  list files being processed in detail
    --warning=KEYWORD              warning control:
-w, --interactive, --confirmation      require confirmation for each operation

Compatibility options:

-o                         equivalent to --old-archive when creating an archive; equivalent to --no-same-owner when extracting an archive

Other options:

-?, --help                 display this help list
    --restrict                 disable certain potentially dangerous options
    --usage                    display a brief usage message
    --version                  print program version

Long options and their corresponding short options have the same required or optional arguments.

Unless a backup suffix is set with --suffix or SIMPLE_BACKUP_SUFFIX, the backup suffix is "~".
Backup can be set to a version control using --backup or VERSION_CONTROL, the possible values are:

none, off        never make backups
t, numbered      make numbered backups
nil, existing    make numbered backups 
if numbered backups already exist, otherwise make simple backups
never, simple    always make simple backups

--quoting-style option has the following valid parameters:

  literal
  shell
  shell-always
  c
  c-maybe
  escape
  locale
  clocale

The default value for this tar command is:
--format=gnu -f- -b20 --quoting-style=escape --rmt-command=/etc/rmt
--rsh-command=/usr/bin/ssh
```

## Common Commands

Compress file without packaging:

```
touch a.c
tar -czvf test.tar.gz a.c   //Compress file a.c to test.tar.gz
```
- z: with gzip attribute
- j: with bz2 attribute
- Z: with compress attribute
- v: display all processes
- O: extract files to standard output

List the content of compressed file:

```
tar -tzvf test.tar.gz
-rw-r--r-- root/root     0 2010-05-24 16:51:59 a.c
```

Extract files:

```
tar -xzvf test.tar.gz
a.c
```