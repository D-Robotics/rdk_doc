---
sidebar_position: 1
---

# netstat

The **netstat command** is used to print the status of the network system in Linux, allowing you to know the network situation of the entire system.

## Syntax

```
netstat [-vWeenNcCF] [<Af>] -r         
netstat {-V|--version|-h|--help}
netstat [-vWnNcaeol] [<Socket> ...]
netstat { [-vWeenNac] -i | [-cnNe] -M | -s [-6tuw] }
```

## Option Description

- `-A`: List the relevant addresses in the network type connection.
- `-r, --route`: Display the routing table and list the routing information of the system.
- `-i, --interfaces`: Display network interface information, including interface names, IP addresses, and other relevant information.
- `-g, --groups`: Display multicast group member information, including which network members are in the multicast group.
- `-s, --statistics`: Display network statistics information, similar to SNMP (Simple Network Management Protocol), providing detailed statistics on network activity.
- `-M, --masquerade`: Display masquerade connection information, usually used in Network Address Translation (NAT) networks.
- `-v, --verbose`: Display detailed information, providing more information to help diagnose network problems.
- `-W, --wide`: Do not truncate the IP address to display complete IP address information.
- `-n, --numeric`: Do not resolve hostnames or port names to display IP addresses, port numbers, and user information in numeric format.
- `--numeric-hosts`: Do not resolve hostnames.
- `--numeric-ports`: Do not resolve port names.
- `--numeric-users`: Do not resolve user names.
- `-N, --symbolic`: Resolve hardware names and display symbolic names for hardware devices.
- `-e, --extend`: Display additional information. Use this option twice to get the maximum detailed information.
- `-p, --programs`: Display the PID (Process Identifier) and program names to show process information related to sockets.
- `-o, --timers`: Display timer information, including the timer status of sockets.
- `-c, --continuous`: Make `netstat` continuously print the selected information every second for continuous monitoring.
- `-l, --listening`: Only display listening server sockets.
- `-a, --all`: Display all sockets, including connected and unconnected ones.
- `-F, --fib`: Display Forwarding Information Base (FIB).
- `-C, --cache`: Display the routing cache instead of the Forwarding Information Base.
- `-Z, --context`: Display SELinux security context for displaying SELinux security information of sockets.
- `-v, --verbose`: Enable verbose output to provide more information to the user about ongoing operations. Particularly useful when dealing with unconfigured address families, providing some useful information.
- `-o, --timers`: Include information related to network timers.
- `-p, --program`: Display the PID and name of the program that each socket belongs to.
- `-l, --listening`: Only display listening sockets. By default, these are omitted.
- `-a, --all`: Display both listening and non-listening sockets. When using the `--interfaces` option, display disabled interfaces.
- `-F`: Print routing information from the FIB (default).
- `-C`: Print routing information from the cache. 

## Common Commands

Display detailed network conditions

```
netstat -a       #List all ports
netstat -at      #List all TCP ports
netstat -au      #List all UDP ports
```

Display current registered UDP connections

```
netstat -nu
```

Display usage of UDP port numbers

```
netstat -apu
```

Display network card list

```
netstat -i
```

Display multicast group relationships

```
netstat -g
```

Display network statistics

```
netstat -s    #Display statistics of all ports
netstat -st   #Display statistics of TCP ports
netstat -su   #Display statistics of UDP ports
```

Display listening sockets

```
netstat -l      #Only display listening ports
netstat -lt     #Only list all listening TCP ports
netstat -lu     #Only list all listening UDP ports
netstat -lx     #Only list all listening UNIX ports
```

Display PID and process name in netstat output

```
netstat -pt
```

`netstat -p` can be used with other options to add "PID/Process Name" to netstat output.

Continuously output netstat information

```
netstat -c   #output network information every second
```

Display kernel routing information

```shell
netstat -r
```

Use `netstat -rn` to display in numerical format, without querying host names.

Find ports that programs are running on

Not all processes can be found, those without permission will not be displayed, view all information with root permission.

```shell
netstat -ap | grep ssh
```

Find processes running on a specific port

```shell
netstat -an | grep ':80'
```

Find process ID by port

```bash
netstat -anp|grep 8081 | grep LISTEN|awk '{printf $7}'|cut -d/ -f1
```