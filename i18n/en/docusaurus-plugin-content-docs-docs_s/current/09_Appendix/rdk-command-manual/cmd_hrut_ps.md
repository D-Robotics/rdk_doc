---
sidebar_position: 1
---

# hrut_ps

The **hrut_ps** command prints process status information not supported by BusyBox's `ps` command, including process ID, parent process ID, priority, memory usage, virtual memory, and more.

## Syntax

```
  hrut_ps
```

## Supported Information

- **pid**: Process ID. Each process in the operating system has a unique identifier called the process ID (pid), which is used to uniquely identify and reference a process within the system.
- **ppid**: Parent Process ID. This indicates the parent process that created the current process.
- **state**: Process state.
  - `I`: Idle.
  - `R`: Running.
  - `S`: Sleeping.
  - `D`: Disk sleeping (uninterruptible sleep).
  - `T`: Stopped.
  - `X`: Dead.
  - `Z`: Zombie.
  - `t`: Tracing stop.
  - `P`: Parked.
- **prio**: Priority. Represents the scheduling priority of the process, usually expressed as a numeric value. A higher number typically indicates higher priority, making the process more likely to receive CPU time slices.
- **nice**: Nice value. Represents the scheduling priority as an integer. A lower nice value indicates higher priority, allowing the process to obtain CPU time more frequently.
- **rt_prio**: Real-time priority. Indicates the priority of real-time processes; a lower value signifies higher real-time priority.
- **policy**: Scheduling policy. Specifies the scheduling policy used for the process, typically a scheduling algorithm such as First-In-First-Out (FIFO), Round Robin (RR), etc.
- **vsize**: Virtual memory size. Represents the size of the virtual memory address space accessible to the process.
- **rss**: Resident Set Size (physical memory usage). Indicates the amount of physical RAM currently allocated and used by the process.
- **comm**: Command name. Contains the name of the command or executable file associated with the process, used to identify the process type or purpose.