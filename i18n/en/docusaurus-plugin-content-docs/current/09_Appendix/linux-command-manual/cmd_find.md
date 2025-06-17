---
sidebar_position: 1
---

# find

The `find` command is used to search for files and directories in a specified directory.

It can use different options to filter and limit the search results. Any string before the parameters is considered as the directory name to search for. If no parameters are set, the `find` command will search for subdirectories and files in the current directory and display all the found subdirectories and files.

## Syntax

```
find [-H] [-L] [-P] [-Olevel] [-D debugopts] [path...] [expression]
```

## Options

**path** is the directory path to search, which can be a directory or file name. Multiple paths can be specified, separated by spaces. If no path is specified, it defaults to the current directory.

**expression** is an optional parameter that specifies the search conditions, such as file names, file types, file sizes, etc.

There are dozens of options that can be used in the expression. Here are the most commonly used ones:

- `-name pattern`: Search by file name, supports wildcard characters `*` and `?`.
- `-iname pattern`: Similar to `-name`, but ignores case differences.
- `-type type`: Search by file type, can be `f` (regular file), `d` (directory), `l` (symbolic link), etc.
- `-size [+-]size[cwbkMG]`: Search by file size, supports using `+` or `-` to specify larger or smaller sizes. Units can be `c` (bytes), `w` (words), `b` (blocks), `k` (KB), `M` (MB), or `G` (GB).
- `-mtime days`: Search by modification time, supports using `+` or `-` to specify days before or after a certain time. `days` is an integer representing the number of days.
- `-user username`: Search by file owner.
- `-group groupname`: Search by file group.

The time parameters used in the `find` command are as follows:

- `-amin n`: Search for files accessed within the last `n` minutes.
- `-atime n`: Search for files accessed within the last `n*24` hours.
- `-cmin n`: Search for files with status changes (e.g., permissions) within the last `n` minutes.
- `-ctime n`: Search for files with status changes (e.g., permissions) within the last `n*24` hours.
- `-mmin n`: Search for files modified within the last `n` minutes.
- `-mtime n`: Search for files modified within the last `n*24` hours.

In these parameters, `n` can be a positive number, negative number, or zero. A positive number represents files modified or accessed within the specified time period, a negative number represents files modified or accessed before the specified time period, and zero represents files modified or accessed at the current time.

For example, `-mtime 0` searches for files modified today, and `-mtime -7` searches for files modified more than a week ago.

Explanation of the time parameter `n`:

- `+n`: Search for files or directories older than `n` days.
- `-n`: Search for files or directories with attribute changes within the last `n` days.
- `n`: Search for files or directories with attribute changes on a specific day `n` days ago.

## Common Commands

List all files and folders in the current directory and subdirectories:

```shell
find .
```

Find a file named file.txt in the current directory:

```
find . -name file.txt
```

List all files with the **.c** file extension in the current directory and its subdirectories:

```
find . -name "*.c"
```

Same as above, but ignore case:

```shell
find . -iname "*.c"
```

List all files in the current directory and its subdirectories:

```
find . -type f
```

Find files larger than 1MB in /home directory:

```
find . -size +1M
```

Search for files smaller than 10KB:

```shell
find . -type f -size -10k
```

Search for files that are exactly 10KB:

```shell
find . -type f -size 10k
```
  - File size units:

    - **b** —— block (512 bytes)

    - **c** —— byte

    - **w** —— word (2 bytes)

    - **k** —— kilobyte

    - **M** —— megabyte

    - **G** —— gigabyte

Find files modified in /var/log directory 7 days ago

```
find /var/log -mtime +7
```

List all files that were last updated 20 days ago in the current directory and its subdirectories, exactly 20 days ago

```
find . -ctime  20
```

List all files that were last updated 20 days ago or earlier in the current directory and its subdirectories

```
find . -ctime  +20
```

List all files that were last updated within the last 20 days in the current directory and its subdirectories

```
find . -ctime  20
```

Find regular files in /var/log directory whose modification time is more than 7 days ago, and prompt before deleting them

```
find /var/log -type f -mtime +7 -ok rm {} \;
```

Find files in the current directory whose owner has read and write permissions, and the group and other users have read permissions

```
find . -type f -perm 644 -exec ls -l {} \;
```

List all ordinary files in the system with a length of 0, and display their complete paths.

```
find / -type f -size 0 -exec ls -l {} \;
```

Search for all files ending with .txt and .pdf in the current directory and subdirectories.

```shell
find . \( -name "*.txt" -o -name "*.pdf" \)
or
find . -name "*.txt" -o -name "*.pdf"
```

Match file paths or filenames.

```shell
find /usr/ -path "*local*"
```

Match file paths based on regular expressions.

```shell
find . -regex ".*\(\.txt\|\.pdf\)$"
```

Same as above, but ignore case.

```shell
find . -iregex ".*\(\.txt\|\.pdf\)$"
```

Negate the parameter, find files in /home that do not end with .txt.

```shell
find /home ! -name "*.txt"
```

Search based on file type.

```shell
find . -type type_parameter
```

- List of type_parameters:
  - **f** for regular files
  - **l** for symbolic links
  - **d** for directories
  - **c** for character devices- **b** block device
  - **s** socket
  - **p** FIFO

Based on directory depth search, with a maximum depth limit of 3

```shell
find . -maxdepth 3 -type f
```

Search for all files that are at least 2 subdirectories deep from the current directory

```shell
find . -mindepth 2 -type f
```

Delete all `.log` files in the current directory

```shell
find . -type f -name "*.log" -delete
```

Search for files with permission 777 in the current directory

```shell
find . -type f -perm 777
```

Find `.conf` files in the current directory whose permissions are not 644

```shell
find . -type f -name "*.conf" ! -perm 644
```

Find all files owned by the user `sunrise` in the current directory

```shell
find . -type f -user sunrise
```

Find all files owned by the group `sunrise` in the current directory

```shell
find . -type f -group sunrise
```

Find all files with owner `root` in the current directory and change the ownership to user `sunrise`

```shell
find . -type f -user root -exec chown sunrise {} \;
```

In the above example, **{}** is used in combination with the **-exec** option to match all files and will be replaced with the respective file name.

Find all the `.txt` files under the `home` directory and delete them:

```shell
find $HOME/. -name "*.txt" -ok rm {} \;
```

In the above example, **-ok** is similar to **-exec** but prompts for confirmation before executing the operation.

Find all `.txt` files in the current directory and concatenate them into a file named `all.txt`:

```shell
find . -type f -name "*.txt" -exec cat {} \; > /all.txt
```

Search for all `.txt` files in the current directory or its subdirectories, but skip the subdirectory `sk`:

```shell
find . -path "./sk" -prune -o -name "*.txt" -print
```

> ⚠️ `./sk` should not be written as `./sk/` or it will have no effect.

Ignore two directories:

```shell
find . \( -path ./sk -o -path ./st \) -prune -o -name "*.txt" -print
```

> ⚠️ If using relative paths, `./` must be added.

To list all files with zero length:

```shell
find . -empty
```

Count the number of lines in code files:

```shell
find . -name "*.c" | xargs cat | grep -v ^$ | wc -l # Code line count, excluding blank lines.
```

Other examples:

```shell
find ~ -name '*jpg' # Find all jpg files in the home directory. The -name parameter allows you to limit the results to files matching the given pattern.
find ~ -iname '*jpg' # -iname is like -name, but it is case insensitive
find ~ \( -iname 'jpeg' -o -iname 'jpg' \) # some images may have the .jpeg extension. Luckily, we can combine patterns with "or" (represented by -o)
find ~ \( -iname '*jpeg' -o -iname '*jpg' \) -type f # What if you have directories that end with jpg? (Why you named a directory bucketofjpg instead of pictures is beyond the scope of this article.) We modify our command with the -type parameter to search for files.
find ~ \( -iname '*jpeg' -o -iname '*jpg' \) -type d # You may also want to find those oddly named directories for future renaming.