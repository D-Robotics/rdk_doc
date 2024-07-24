---
sidebar_position: 1
---

# zip

The **zip command** can be used to decompress files or perform packaging operations on files. zip is a widely used compression program, and files compressed with it will produce a compressed file with the extension `.zip`.

## Syntax

```
zip [-options] [-b path] [-t mmddyyyy] [-n suffixes] [zipfile list] [-xi list]
```

- **zipfile list**: Specifies the zip file to be created.
- File list: Specifies the list of files to be compressed.

## Option Explanation

- `-0`: Only store data without compression.
- `-1`: Faster compression speed.
- `-9`: Better compression quality.
- `-'compression efficiency'`: Compression efficiency is a value between 1-9.
- `-@`: Reads file names from standard input.
- `-A`: Adjusts executable self-extracting files.
- `-b path`: Specifies the temporary directory for storing files.
- `-c`: Adds a comment to each compressed file.
- `-d`: Deletes entries in the compressed file.
- `-D`: Does not add directory entries.
- `-e`: Encrypts.
- `-f`: This option is similar to specifying "-u", but not only updates existing files. If some files do not exist in the compressed file, this option will add them to the compressed file.
- `-F`: Attempts to repair damaged compressed files (-FF attempts a stricter repair).
- `-h2`: Displays more help.
- `-i`: Only compresses files that meet the conditions.
- `-j`: Only saves file names and their contents without storing any directory names.
- `-J`: Does not record the compression file prefix (used for self-extracting files).
- `-l`: Converts LF to CR LF (-ll converts CR LF to LF).
- `-m`: Moves to the compressed file (deletes the operating system file).
- `-n`: Does not compress files with these suffixes.
- `-o`: Makes the compressed file have the same date as the newest entry.
- `-q`: Runs in quiet mode.
- `-r`: Recursively into directories.
- `-s`: Changes the delay time between two refreshes, in seconds (if there are decimals, in milliseconds). Entering a value of 0 means the system will refresh continuously, the default value is 5 seconds.
- `-T`: Tests the integrity of the compressed file.
- `-u`: Updates (only includes changed or new files).
- `-v`: Detailed operation, prints version information.
- `-X`: Excludes additional file attributes.
- `-y`: Directly saves symbolic links instead of the files they point to. This parameter is only effective on systems like UNIX.
- `-z`: Adds a comment to the compressed file.

## Common Commands

Pack all files and folders under the `/app` directory into a `app.zip` file in the current directory:

```
zip -q -r app.zip /app
```

If we are in the `/app` directory, we can execute the following command:

```
zip -q -r app.zip *
```

Delete the file `a.c` from the compressed file `cp.zip`:

```
zip -dv cp.zip a.c
```