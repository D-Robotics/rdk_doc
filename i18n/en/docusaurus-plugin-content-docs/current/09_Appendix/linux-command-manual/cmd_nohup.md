---
sidebar_position: 1
---

# nohup

**nohup** (short for no hang up) is used to run a command in the background without being affected by terminal closures.

By default (when not redirected), it outputs a file named nohup.out to the current directory. If the nohup.out file in the current directory is not writable, the output will be redirected to the `$HOME/nohup.out` file. If no file can be created or opened for appending, the specified command in the "command" parameter will not be executable. If the standard error is a terminal, all output from the specified command that is written to the standard error will be redirected to the same file descriptor as the standard output.

## Syntax

```
nohup COMMAND [ARG]... [　& ]
nohup OPTION
```

**COMMAND**: The command to be executed.

**ARG**: Any additional parameters that can be used to specify an output file.

**&**: Allows the command to be executed in the background, even after the terminal is closed.

## Option explanation

- `--help`: Display help information.
- `--version`: Display version information.

## Common commands

The following command executes the runoob.sh script in the background under the root directory:

```
nohup /root/runoob.sh &
```

To stop the execution, you need to use the following command to find the PID of the running script using nohup, and then use the kill command to delete it:

```
ps -aux | grep "runoob.sh" 
```

The following command executes the runoob.sh script in the background under the root directory and redirects the input to the runoob.log file:

```
nohup /root/runoob.sh > runoob.log 2>&1 &
```

Explanation of `2>&1`:

Redirect standard error 2 to standard output &1, and then redirect standard output &1 to the runoob.log file.

- 0 - stdin (standard input)
- 1 - stdout (standard output)
- 2 - stderr (standard error output)