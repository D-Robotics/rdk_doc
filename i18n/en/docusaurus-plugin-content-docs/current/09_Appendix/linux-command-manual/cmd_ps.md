---
sidebar_position: 1
---

# ps

The **ps command** is used to display the current system process status. Ps is the most basic and powerful process viewing command. By using this command, you can determine which processes are running, their running status, whether processes have ended, if any processes are in a zombie state, and which processes are consuming excessive resources.

## Syntax

```
ps [options]
```

## Option Description

**Basic options:**

- `-A, -e`: Display all processes.
- `-a`: Display all processes with a terminal (tty), excluding session leaders.
- `a`: Display all processes with a terminal (tty), including processes of other users.
- `-d`: Display all processes except session leaders.
- `-N, --deselect`: Deselect (invert) the processes.
- `r`: Display only running processes.
- `T`: Display all processes associated with the current terminal.
- `x`: Display processes without a control terminal.

**Selection by list:**

- `-C <command>`: Select processes by command name.
- `-G, --Group <GID>`: Select processes by real group ID or group name.
- `-g, --group <group>`: Select processes by session or effective group name.
- `-p, p, --pid <PID>`: Select processes by process ID.
- `--ppid <PID>`: Select processes by parent process ID.
- `-q, q, --quick-pid <PID>`: Quick mode, select processes by process ID.
- `-s, --sid <session>`: Select processes by session ID.
- `-t, t, --tty <tty>`: Select processes by terminal.
- `-u, U, --user <UID>`: Select processes by effective user ID or username.
- `-U, --User <UID>`: Select processes by real user ID or username.

**Output formats:**

- `-F`: Display additional detailed information.
- `-f`: Full format, including the command line.
- `f, --forest`: Display the process tree in ASCII art.
- `-H`: Display process hierarchy.
- `-j`: Job format.
- `j`: BSD job control format.
- `-l`: Long format.
- `l`: BSD long format.
- `-M, Z`: Add security data (for SELinux).
- `-O <format>`: Use default column preloading.
- `-O <format>`: Preload columns in BSD style.
- `-o, -o, --format <format>`: User-defined format.
- `s`: Signal format.
- `u`: User-oriented format.
- `v`: Virtual memory format.
- `X`: Register format.
- `-y`: Do not display flags, display RSS and address (with -l).
- `--context`: Display security context (for SELinux).
- `--headers`: Repeat header lines on each page.
- `--no-headers`: Do not print headers at all.
- `--cols, --columns, --width <num>`: Set screen width.
- `--rows, --lines <num>`: Set screen height.

**Show threads:**

- `H`: Show threads as if they were processes.
- `-L`: May include LWP and NLWP columns.
- `-m, -m`: Display threads after processes.
- `-T`: May include SPID column.

**Miscellaneous options:**

- `-c`: Display scheduling class with -l option.
- `c`: Display the real command name.
- `e`: Display the environment after the command.
- `k, --sort`: Specify sort order, e.g.: [+|-]key[,[+|-]key[,...]].
- `L`: Display format specifiers.
- `n`: Display numeric user ID and wchan.
- `S, --cumulative`: Include some terminated child process data.
- `-y`: Do not display flags, display RSS (only with -l).
- `-V, -V, --version`: Display version information and exit.
- `-w, -w`: Unlimited output width.

**Help options:**

- `--help <simple|list|output|threads|misc|all>`: Display help and exit. Different help modes can be selected.

## Common commands

List the PID and related information belonging to the currently logged-in user:

```
sunrise@ubuntu:~$ ps -l
F S   UID     PID    PPID  C PRI  NI ADDR SZ WCHAN  TTY          TIME CMD
4 S  1000    4295    4294  8  80   0 -  3304 do_wai pts/0    00:00:00 bash
0 R  1000    4304    4295  0  80   0 -  3504 -      pts/0    00:00:00 ps
```
- `F`: represents the flag of this program, where 4 indicates that the user is a super user.
- `S`: represents the STAT (state) of this program, and the meanings of different STATs will be explained below.
  - `S`: Sleeping - the process is running.
  - `R`: Running - the process is running or ready to run.
  - `D`: Uninterruptible Sleep - the process is uninterruptible.
  - `T`: Stopped - the process has stopped.
  - `Z`: Zombie - the process is a zombie process.
  - `t`: Traced or stopped - the process is being traced or has stopped.
  - `P`: Parked - the process is being traced or has stopped, but it is waiting.
- `UID`: UID of the process, indicating the user who runs the process.
- `PID`: Process ID, a unique identifier assigned to each process by the operating system.
- `PPID`: Parent Process ID, indicating the ID of the parent process that launched the current process.
- `C`: CPU usage percentage, indicating the percentage of CPU time occupied by the process.
- `PRI`: Process priority.
- `NI`: Nice value of the process, usually used to adjust the process priority.
- `ADDR`: Address space of the process. This is a kernel function that indicates the part of the program in memory. If the program is running, it is usually "-".
- `SZ`: Virtual memory size of the process in pages.
- `WCHAN`: The event or lock that the process is currently waiting for. If it is "-", it means the process is running.
- `TTY`: The terminal associated with the process (if any).
- `TIME`: CPU time the process has been running.
- `CMD`: Command line of the process.

List all the currently running programs in memory:

```
sunrise@ubuntu:~$ ps aux
USER         PID %CPU %MEM    VSZ   RSS TTY      STAT START   TIME COMMAND
root           1  0.4  0.4 167592 10076 ?        Ss   10:16   0:02 /sbin/init
root           2  0.0  0.0      0     0 ?        S    10:16   0:00 [kthreadd]
root           4  0.0  0.0      0     0 ?        I<   10:16   0:00 [kworker/0:0H]
root           5  0.0  0.0      0     0 ?        I    10:16   0:00 [kworker/u8:0]
```

- `USER`: Represents the user to which the process belongs, i.e., the user who runs the process.
- `PID`: represents the process ID, a unique identifier used to identify each process.
- `%CPU`: represents the CPU usage of the process, i.e., the percentage of CPU time the process uses out of the total CPU time.
- `%MEM`: represents the memory usage of the process, i.e., the percentage of physical memory the process uses out of the total physical memory.
- `VSZ`: represents the virtual memory size of the process (Virtual Set Size), i.e., the size of the virtual memory the process can access, usually measured in kilobytes (KB).
- `RSS`: represents the physical memory size of the process (Resident Set Size), i.e., the current amount of physical memory the process occupies, usually measured in kilobytes (KB).
- `TTY`: represents the terminal associated with the process. If the process is not associated with a terminal, it displays "?".
- `STAT`: represents the state of the process, usually including some of the following states:
  - `R`: Running
  - `S`: Sleeping
  - `D`: Uninterruptible Sleep
  - `Z`: Zombie
  - `T`: Stopped
  - Other states may also exist, and their specific meanings may vary depending on the operating system.
- `START`: represents the start time of the process, usually in the format of hour:minute.
- `TIME`: represents the CPU time the process has used, usually in the format of hour:minute:second.
- `COMMAND`: represents the command line of the process, i.e., the command and its arguments that the process is executing.List similar program tree displays

```shell
sunrise@ubuntu:~$ ps -axjf
   PPID     PID    PGID     SID TTY        TPGID STAT   UID   TIME COMMAND
      1    2973    2973    2973 ?             -1 Ss       0   0:00 sshd: /usr/sbin/sshd -D [listener] 0 of 10-100 startups
   2973    4067    4067    4067 ?             -1 Ss       0   0:00  \_ sshd: root@pts/0
   4067    4239    4239    4239 pts/0       4364 Ss       0   0:00  |   \_ -bash
   4239    4294    4294    4239 pts/0       4364 S        0   0:00  |       \_ su sunrise
   4294    4295    4295    4239 pts/0       4364 S     1000   0:00  |           \_ bash
   4295    4364    4364    4239 pts/0       4364 R+    1000   0:00  |               \_ ps -axjf
   2973    4069    4069    4069 ?             -1 Ss       0   0:00  \_ sshd: root@notty
   4069    4242    4242    4242 ?             -1 Ss       0   0:00      \_ /usr/lib/openssh/sftp-server
```

Other commands

```
ps axo pid,comm,pcpu # View PID, name, and CPU usage of processes
ps aux | sort -rnk 4 # Sort processes by memory usage
ps aux | sort -nk 3  # Sort processes by CPU usage
ps -A # Show all process information
ps -u root # Show information of a specific user
ps -efL # View thread count
ps -e -o "%C : %p :%z : %a"|sort -k5 -nr # View processes and sort by memory usage
ps -ef # Show all process information, including command lines
ps -ef | grep ssh # Common usage of ps and grep, find specific processes
ps -C nginx # Search processes by name or command
ps aux --sort=-pcpu,+pmem # Sort by CPU or memory usage, -descending, +ascending
ps -f --forest -C nginx # Display process hierarchy in tree style
ps -o pid,uname,comm -C nginx # Display child processes of a parent process
ps -e -o pid,uname=USERNAME,pcpu=CPU_USAGE,pmem,comm # Redefine labels
ps -e -o pid,comm,etime # Show duration of process running
ps -aux | grep named # View detailed information of named process
ps -o command -p 91730 | sed -n 2p # Get service name by process ID
```