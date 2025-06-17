---
sidebar_position: 1
---

# hrut_ps

The **hrut_ps** command prints process information that is not supported by the `ps` command in busybox, including process ID, parent process ID, priority, memory, virtual memory, etc.

## Syntax

```
hrut_ps
```

## Supported Information

- **pid**: Process ID. Each process in the operating system has a unique identifier called the process ID (pid). It is used to uniquely identify and identify a process in the system.
- **ppid**: Parent Process ID. It indicates the parent process that created this process.
- **state**: Running state.
  - `I`: Idle.
  - `R`: Running.
  - `S`: Sleeping.
  - `D`: Disk sleeping.
  - `T`: Stopped.
  - `X`: Dead.
  - `Z`: Zombie.
  - `t`: Tracing stop.
  - `P`: Parked.
- **prio**: Priority. It represents the scheduling priority of the process, usually a numerical value. A higher value indicates a higher priority, and the process may be more likely to obtain CPU time slices.
- **nice**: Scheduling priority. It represents the scheduling priority of the process, usually an integer value. A lower nice value indicates a higher priority, allowing the process to get CPU time more frequently.
- **rt_prio**: Real-time priority. It represents the priority of real-time processes. A lower value indicates a higher real-time priority.
- **policy**: Scheduling policy. It represents the scheduling policy of the process, usually a scheduling algorithm, such as First-In-First-Out (FIFO), Round Robin, etc.
- **vsize**: Virtual memory size. It represents the size of the process's virtual memory, which is the size of the virtual address space that the process can access.
- **rss**: Resident Set Size. It represents the current occupied physical memory size of the process, which is the actual amount of physical RAM allocated to the process.
- **comm**: Command name. It contains the command name of the process or the name of the executable file, which is used to identify the type or purpose of the process.