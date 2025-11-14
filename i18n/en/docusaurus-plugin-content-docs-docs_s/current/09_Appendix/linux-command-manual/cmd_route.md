---
sidebar_position: 1
---

# route

The **route** command is used to display and set the network routing table in the Linux kernel. The routes set by the route command are mainly static routes. To achieve communication between two different subnets, a router connecting both networks or a gateway located in both networks is needed.

Setting routes in Linux systems is usually done to solve the following problems:

If a Linux system is in a LAN (Local Area Network) with a gateway that allows machines to access the Internet, the IP address of this machine needs to be set as the default route for the Linux machine. It should be noted that adding routes directly through the command line using the route command will not be permanently saved. The route will become invalid when the network card or the machine is restarted. To ensure that the route is permanently set, the route command can be added to the `/etc/rc.local` file.

## Syntax

```
route [-nNvee] [-FC] [<AF>]           List kernel routing tables
route [-v] [-FC] {add|del|flush} ...  Modify routing table for AF.
```

- `-A`: Set the address type.
- `-v, --verbose`: Display detailed information.
- `-n, --numeric`: Do not perform DNS reverse lookup and display IP addresses in numeric form.
- `-e, --extend`: Display the routing table in netstat format.
- `-F, --fib`: Display the forwarding information base (default).
- `-C, --cache`: Display the route cache instead of the forwarding information base.
- `-net`: Show the routing table for a network.
- `-host`: Show the routing table for a host.

## Options

- `add`: Used to add a specified routing record, routing the specified destination network or host to the specified network interface.
- `del`: Used to delete a specified routing record.
- `target`: Specifies the destination network or host.
- `gw`: Used to set the default gateway.
- `mss`: Set the maximum segment size (MSS) for TCP.
- `window`: Specify the TCP window size for TCP connections through the routing table.
- `dev`: Specify the network interface represented by the routing record.

## Commonly Used Commands

Display the current routing table:

```
root@ubuntu:~# route
Kernel IP routing table
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
default         192.168.0.1     0.0.0.0         UG    600    0        0 wlan0
default         192.168.1.1     0.0.0.0         UG    700    0        0 eth0
192.168.0.0     0.0.0.0         255.255.255.0   U     600    0        0 wlan0
192.168.1.0     0.0.0.0         255.255.255.0   U     700    0        0 eth0
root@ubuntu:~# route -n
Kernel IP routing table
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
0.0.0.0         192.168.0.1     0.0.0.0         UG    600    0        0 wlan0
0.0.0.0         192.168.1.1     0.0.0.0         UG    700    0        0 eth0
192.168.0.0     0.0.0.0         255.255.255.0   U     600    0        0 wlan0
192.168.1.0     0.0.0.0         255.255.255.0   U     700    0        0 eth0
```

The Flags column indicates the status of the network node, and the Flags symbols are explained as follows:

- `U`: Up, indicating that the route is currently in an active state.
- `H`: Host, indicating that the gateway is a host.
- `G`: Gateway, indicating that the gateway is a router.
- `R`: Reinstate Route, indicating that the route has been initialized again using dynamic routing.
- `D`: Dynamically, indicating that the route has been dynamically written.
- `M`: Modified, indicating that the route has been dynamically modified by the routing daemon or router.
- `!`: Indicates that the route is currently in an inactive state.

Adding a gateway / setting a gateway

```
route add -net 192.168.2.0 netmask 255.255.255.0 dev eth0
```

Blocking a route

```shell
route add -net 192.168.2.0 netmask 255.255.255.0 reject
```

Deleting a route record

```shell
route del -net 192.168.2.0 netmask 255.255.255.0
route del -net 192.168.2.0 netmask 255.255.255.0 reject
```

Deleting and adding a default gateway

```shell
route del default gw 192.168.2.1
route add default gw 192.168.2.1
```