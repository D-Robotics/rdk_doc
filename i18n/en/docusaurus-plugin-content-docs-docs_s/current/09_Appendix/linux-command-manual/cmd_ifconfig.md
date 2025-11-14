---
sidebar_position: 1
---

# ifconfig

**ifconfig** is a command used to configure and manage network interfaces. It allows users to view and modify the configuration of network interfaces, including IP address, subnet mask, MAC address, MTU, broadcast address, point-to-point address, etc.

## Syntax

```
ifconfig [-a] [-v] [-s] <interface> [[<AF>] <address>]
  [add <address>[/<prefixlen>]]
  [del <address>[/<prefixlen>]]
  [[-]broadcast [<address>]]  [[-]pointopoint [<address>]]
  [netmask <address>]  [dstaddr <address>]  [tunnel <address>]
  [outfill <NN>] [keepalive <NN>]
  [hw <HW> <address>]  [mtu <NN>]
  [[-]trailers]  [[-]arp]  [[-]allmulti]
  [multicast]  [[-]promisc]
  [mem_start <NN>]  [io_addr <NN>]  [irq <NN>]  [media <type>]
  [txqueuelen <NN>]
  [[-]dynamic]
  [up|down] ...
```

## Option Description

- `ifconfig`: displays all configured and active network interfaces along with their status.

- `ifconfig -a`: displays all network interfaces, including those that are not active.
- `ifconfig <interface>`: displays the configuration of the specified network interface.
- `ifconfig <interface> up`: activates the specified network interface.
- `ifconfig <interface> down`: deactivates the specified network interface.
- `ifconfig <interface> add <address>`: adds an IP address to the specified network interface.
- `ifconfig <interface> del <address>`: removes an IP address from the specified network interface.
- `ifconfig <interface> netmask <address>`: sets the subnet mask for the specified network interface.
- `ifconfig <interface> broadcast <address>`: sets the broadcast address.
- `ifconfig <interface> pointopoint <address>`: sets the point-to-point address.
- `ifconfig <interface> hw <HW> <address>`: sets the MAC address.
- `ifconfig <interface> mtu <NN>`: sets the MTU (Maximum Transmission Unit).
- `ifconfig <interface> arp`: enables ARP (Address Resolution Protocol).
- `ifconfig <interface> promisc`: enables promiscuous mode, which allows receiving all packets passing through the network interface.
- `ifconfig <interface> multicast`: enables multicast mode.
- `ifconfig <interface> dynamic`: enables dynamic configuration.
- `ifconfig -s`: displays network interface information in a concise format.
- `ifconfig -v`: displays detailed information.

## Common Commands
Common commands

```
ifconfig   # Network interface in active state
ifconfig -a  # All configured network interfaces, regardless of whether they are active
ifconfig eth0  # Display network card information for eth0
```

Start and stop specified network card

```shell
ifconfig eth0 up
ifconfig eth0 down
```

Configure IP address

```shell
ifconfig eth0 192.168.1.10
ifconfig eth0 192.168.1.10 netmask 255.255.255.0
ifconfig eth0 192.168.1.10 netmask 255.255.255.0 broadcast 192.168.1.255
```