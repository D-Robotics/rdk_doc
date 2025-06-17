---
sidebar_position: 1
---

# top

The **top command** allows you to dynamically view the overall operation of the system in real time. It is a practical tool that combines multiple information monitoring system performance and operational information. It provides a dynamic and interactive real-time view that displays overall performance information of the system and relevant information about running processes.

## Syntax

```
top -hv | -bcEHiOSs1 -d secs -n max -u|U user -p pid(s) -o field -w [cols]
```

## Option Explanation

- `-b`: Run in batch mode and directly output the results to a file.
- `-c`: Display the complete command line without truncation.
- `-d <seconds>`: Screen refresh interval time.
- `-I`: Ignore idle processes.
- `-s`: Secure mode.
- `-S`: Accumulatively display the CPU usage time of processes.
- `-i`: Do not display idle or useless processes.
- `-u <username>`: Specify the username.
- `-p <pid>`: Specify the process.
- `-n <number>`: Number of times to loop display.
- `-H`: Display resource occupation of all threads.

## top Interactive Commands

Some interactive commands that can be used during the execution of the top command. These commands are single-letter, and if the -s option is used in the command line, some commands may be blocked.

- `h`: Display the help screen, providing a brief summary of commands.
- `k`: Terminate a process.
- `i`: Ignore idle and zombie processes (toggle command).
- `q`: Quit the program.
- `r`: Reschedule the priority of a process.
- `S`: Switch to cumulative mode.
- `s`: Change the delay time between two refreshes, in seconds (if decimals are used, in milliseconds). Entering a value of 0 will continuously refresh the system. The default value is 5 seconds.
- `f` or `F`: Add or remove items from the current display.
- `o` or `O`: Change the order of displayed items.
- `l`: Switch to display average load and boot time information.
- `m`: Switch to display memory information.
- `t`: Switch to display process and CPU status information.
- `c`: Switch to display command names and complete command lines.
- `M`: Sort by resident memory size.
- `P`: Sort by CPU usage percentage.
- `T`: Sort by time/accumulative time.
- `w`: Write current settings to the ~/.toprc file.

## Display Information

```
top - 14:55:57 up  1:03,  2 users,  load average: 0.62, 0.55, 0.45
Tasks: 158 total,   1 running, 157 sleeping,   0 stopped,   0 zombie
%Cpu(s):  3.7 us,  9.3 sy,  0.0 ni, 86.0 id,  0.0 wa,  0.0 hi,  0.9 si,  0.0 st
MiB Mem :   1982.2 total,    778.4 free,    645.6 used,    558.1 buff/cache
MiB Swap:      0.0 total,      0.0 free,      0.0 used.   1307.2 avail Mem

    PID USER      PR  NI    VIRT    RES    SHR S  %CPU  %MEM     TIME+ COMMAND
   4496 root      20   0   14660   3092   2628 R  23.8   0.2   0:00.13 top
   3032 root      20   0       0      0      0 S  14.3   0.0   0:26.43 RTW_CMD_THREAD
      1 root      20   0  167580  10200   7216 S   0.0   0.5   0:03.09 systemd
      2 root      20   0       0      0      0 S   0.0   0.0   0:00.00 kthreadd
      4 root       0 -20       0      0      0 I   0.0   0.0   0:00.00 kworker/0:0H
      6 root       0 -20       0      0      0 I   0.0   0.0   0:00.00 mm_percpu_wq
      7 root      20   0       0      0      0 S   0.0   0.0   0:00.13 ksoftirqd/0
```

System Information:

- uptime: The system's uptime and average load.
- tasks: The number of running processes and sleeping processes.
- CPU: The overall CPU usage and usage of each core.
- Memory: The overall memory usage, free memory, and memory used for buffering and caching.

Process Information:

- PID: The process identifier.
- USER: The username of the running process.
- PR (Priority): The priority of the process.
- NI (Nice Value): The priority adjustment value of the process.
- VIRT (Virtual Memory): The size of the process's virtual memory.
- RES (Resident Memory): The amount of physical memory actually used by the process.
- SHR (Shared Memory): The amount of memory shared by the process.
- %CPU: The percentage of CPU usage by the process.
- %MEM: The percentage of memory usage by the process.
- TIME+: The cumulative CPU time of the process.

Features and Interactions:

- Key Commands: When running top, certain key commands can be used for operations, such as pressing "k" to terminate a process and pressing "h" to display help information.
- Sorting: Processes can be sorted by CPU usage, memory usage, process ID, etc.
- Refresh Rate: The refresh rate of top can be set to dynamically view system information.

## Common Commands

Display Process Information

```
top
```

Show the complete command

```
top -c
```

Display program information in batch mode

```
top -b
```

Display program information in cumulative mode

```
top -S
```

Set the number of information updates

```
top -n 2   # means stop updating display after 2 updates
```

Set the information update interval

```
top -d 3  # means update every 3 seconds
```

Display specific process information

```
top -p 139  # display process information for process number 139, including CPU and memory usage
```

Exit after 10 updates

```
top -n 10
```

Users will not be able to use interactive commands to control processes

```
top -s
```